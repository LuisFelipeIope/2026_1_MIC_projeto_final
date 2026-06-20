/*
Alterações data 19/06/2026
PROJETO FINAL DA UC DE MICROCONTROLADORES:
CONTROLE DE TEMPERATURA COM LM35

Alunos: Luis Iope e Matheus Machado

N�cleo obrigat�rio:
Leitura do LM35 via ADC e convers�o para �C
Sa�da PWM controlando a pot�ncia da l�mpada
UART TX (envio da temperatura)
UART RX (recebimento de comandos)
Controle ON-OFF

Desafios extra:
-Setpoint via bot�es f�sicos (feito)
@Luis adicionar aqui os desafios que conseguirmos implementar
-Display local (LCD ou 7 segmentos)
-Amostragem por timer/interrupção
*/

#define F_CPU 16000000
#include <xc.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/eeprom.h> //Biblioteca pra leitura e escrita na EEPROM interna

//Vari�veis globais:
uint16_t gTemperatura = 0; //Temperatura atual em �C
uint16_t gSetpoint = 25; //Temperatura desejada em �C
uint8_t gLampadaLigada = 0; //Flag para l�mpada (0 para desligada, 1 para ligada)
uint16_t gLimiarAlarme = 25; //Temperatura que dispara o alarme em °C (editável via serial: AL=)
uint8_t gAlarmeAtivo = 0; //Flag que indica se o alarme já foi enviado (evita ficar mandando toda hora)

uint8_t *gEepromIndice = (uint8_t*)0; //Endereço onde fica salvo o índice atual do buffer circular
uint8_t *gEepromHistorico = (uint8_t*)1; //Endereço inicial do vetor de histórico (ocupa 10 bytes a partir daqui)
uint8_t gHistIndice = 0; //Índice atual do buffer circular (cópia em RAM do que está na EEPROM)
uint16_t gUltimaTempGravada = 0xFFFF; //Guarda a última temperatura gravada, pra só escrever na EEPROM quando o valor mudar

#define UART_BUFFER_SIZE 16 //Texto recebido pela uart de no m�x 16 caracteres
#define HISTORICO_TAMANHO 10 //Quantidade de amostras guardadas no histórico circular da EEPROM

//Vari�veis pra comunica��o serial:
volatile char gUartBuffer[UART_BUFFER_SIZE]; //string que guarda os caracteres que chegam
volatile uint8_t gUartIndex = 0; //guarda posi��o atual onde o pr�ximo caractere ser� salvo no vetor
volatile uint8_t gComandoPronto = 0; //flag que vira 1 quando o usu�rio aperta enter no terminal

//Variável pra base de tempo do controle:
volatile uint8_t gTick = 0; /*flag que o Timer1 seta a cada 250ms, avisa o loop principal que chegou 
                            a hora de ler o sensor e atualizar o controle*/

/*
Pinos do display LCD 16x2. Todos os 6 sinais que
precisamos controlar (RS, EN, D4, D5, D6, D7) estão no PORTB, que está
totalmente livre no projeto. O pino R/W do LCD fica ligado direto no GND
na fiação (não precisamos ler nada do display, só escrever), por isso
ele não aparece aqui como pino controlado pelo micro.
*/
#define LCD_PORT PORTB
#define LCD_DDR  DDRB
#define LCD_RS PB0 //seleciona se o que está sendo mandado é comando ou caractere
#define LCD_EN PB1 //pulso que avisa o LCD pra ler o dado que está nos pinos D4-D7
#define LCD_D4 PB2
#define LCD_D5 PB3
#define LCD_D6 PB4
#define LCD_D7 PB5

/*
Função que gera o pulso no pino EN (Enable) do LCD. É esse pulso que avisa
o controlador do display "os 4 bits que estão nas linhas D4-D7 agora estão
prontos, pode ler". Sobe o pino, espera 1 microssegundo (tempo mínimo), desce o pino e
espera mais um pouco pra dar tempo do controlador processar o dado antes
do próximo nibble chegar.
*/
void lcd_pulse_enable(void)
{
	LCD_PORT |= (1<<LCD_EN);
	_delay_us(1);
	LCD_PORT &= ~(1<<LCD_EN);
	_delay_us(100);
}

/*
Função que manda 4 bits (um nibble) pro LCD de uma vez. Como o display
está ligado em modo 4 bits, cada byte que queremos enviar (comando ou
caractere) precisa ser quebrado em dois nibbles e mandado em duas
chamadas dessa função. Aqui a gente liga ou desliga cada um dos 4 pinos
de dados (D4 a D7) conforme o bit correspondente do nibble recebido, e
depois chama o pulso de EN pra avisar o display que o dado está pronto.
*/
void lcd_send_nibble(uint8_t tNibble)
{
	if (tNibble & 0x01) LCD_PORT |= (1<<LCD_D4); else LCD_PORT &= ~(1<<LCD_D4);
	if (tNibble & 0x02) LCD_PORT |= (1<<LCD_D5); else LCD_PORT &= ~(1<<LCD_D5);
	if (tNibble & 0x04) LCD_PORT |= (1<<LCD_D6); else LCD_PORT &= ~(1<<LCD_D6);
	if (tNibble & 0x08) LCD_PORT |= (1<<LCD_D7); else LCD_PORT &= ~(1<<LCD_D7);
	lcd_pulse_enable();
}

/*
Função que envia um comando pro LCD (por exemplo: limpar a tela, posicionar
o cursor, ligar o display). A diferença entre mandar um comando e mandar
um caractere pra aparecer na tela é só o estado do pino RS: aqui ele é
colocado em nível baixo (RS=0) antes de mandar os dois nibbles do byte
(primeiro os 4 bits mais significativos, depois os 4 menos significativos).
*/
void lcd_command(uint8_t tCmd)
{
	LCD_PORT &= ~(1<<LCD_RS);
	lcd_send_nibble(tCmd >> 4);
	lcd_send_nibble(tCmd & 0x0F);
	_delay_us(50);
}

/*
Função que envia um caractere pra aparecer na tela do LCD. É praticamente
igual à lcd_command, só que aqui o pino RS vai pra nível alto (RS=1) antes
de mandar os nibbles, avisando o controlador do display que o que está
chegando é um dado (caractere) e não um comando.
*/
void lcd_data(uint8_t tDado)
{
	LCD_PORT |= (1<<LCD_RS);
	lcd_send_nibble(tDado >> 4);
	lcd_send_nibble(tDado & 0x0F);
	_delay_us(50);
}

/*
Função que recebe um ponteiro para um texto (string) e vai enviando
caractere por caractere usando a função lcd_data até encontrar o fim
do texto (\0). É o mesmo princípio da uart_print, só que escrevendo
no display em vez de mandar pela serial.
*/
void lcd_string(const char *tStr)
{
	while (*tStr)
	lcd_data(*tStr++);
}

/*
Função que posiciona o cursor do LCD numa linha e coluna específicas.
O controlador guarda a primeira linha a partir do endereço 0x80
e a segunda linha a partir do endereço 0xC0 (de acordo com a tabela de
endereços do display). Somando a coluna desejada a esses endereços base,
a gente calcula o endereço exato e manda como comando.
*/
void lcd_set_cursor(uint8_t tLinha, uint8_t tColuna)
{
	uint8_t tEndereco = (tLinha == 0) ? (0x80 + tColuna) : (0xC0 + tColuna);
	lcd_command(tEndereco);
}

/*
Função que configura o display do zero. Quando o LCD liga, ele pode estar
em qualquer estado interno (não dá pra saber se estava em modo 4 ou 8 bits
de uma utilização anterior), então existe uma sequência específica exigida
pelo datasheet pra forçar um "reset por software": manda 0x03
três vezes com pausas específicas entre elas, e só depois manda 0x02 pra
travar o controlador em modo 4 bits de verdade. Depois disso configuramos:
2 linhas com fonte 5x8 pixels (0x28), limpamos a tela (0x01), definimos que
o cursor deve avançar a cada caractere escrito (0x06), e ligamos o display
sem mostrar o cursor piscando (0x0C).
*/

void lcd_init(void)
{
	LCD_DDR |= (1<<LCD_RS)|(1<<LCD_EN)|(1<<LCD_D4)|(1<<LCD_D5)|(1<<LCD_D6)|(1<<LCD_D7);

	_delay_ms(50); //espera estabilizar a alimentação

	LCD_PORT &= ~(1<<LCD_RS);

	//Sequência de "reset por software" exigida pelo datasheet do HD44780
	lcd_send_nibble(0x03);
	_delay_ms(5);
	lcd_send_nibble(0x03);
	_delay_us(150);
	lcd_send_nibble(0x03);
	_delay_us(150);
	lcd_send_nibble(0x02); //agora sim entra em modo 4 bits

	lcd_command(0x28); //4 bits, 2 linhas, fonte 5x8
	lcd_command(0x08); //display off
	lcd_command(0x01); //clear display
	_delay_ms(2);
	lcd_command(0x06); //entry mode: incrementa cursor, sem deslocar tela
	lcd_command(0x0C); //display on, cursor off, blink off
}

/*
Função que limpa a tela do LCD. Manda o comando de clear (0x01) e espera
2 milissegundos, porque esse comando em especial demora mais pra ser
executado pelo controlador do que os outros comandos (o datasheet exige
essa pausa maior).
*/
void lcd_clear(void)
{
	lcd_command(0x01);
	_delay_ms(2);
}

/*
Função que monta as duas linhas do display com a temperatura atual e o
setpoint, usando sprintf pra formatar os números numa string e depois
mandando pro LCD com lcd_string. O %3u (largura fixa de 3 dígitos) é
proposital: se a temperatura cair de 100 para 99, por exemplo, o terceiro
dígito antigo é sobrescrito por um espaço automaticamente, em vez de
deixar um número fantasma na tela, sem precisar limpar a tela inteira
(o que causaria um piscar visível a cada atualização). O '*' no canto
da segunda linha é só um indicador visual de lâmpada ligada/desligada.
*/
void atualizar_display(void)
{
	char tLinha[17];

	lcd_set_cursor(0, 0);
	sprintf(tLinha, "Temp:%3u C", gTemperatura);
	lcd_string(tLinha);

	lcd_set_cursor(1, 0);
	sprintf(tLinha, "Set :%3u C  %c", gSetpoint, gLampadaLigada ? '*' : ' ');
	lcd_string(tLinha);
}

#define FILTRO_SHIFT 3 //define o peso do filtro (alpha = 1/8). Quanto maior esse número, mais suave e mais lento fica o filtro

//Variáveis do filtro digital:
static uint16_t gAcumuladorFiltro = 0; //guarda o valor filtrado já "ampliado" (multiplicado por 8), pra fazer a conta toda em números inteiros
static uint8_t gFiltroInicializado = 0; //flag que garante que o filtro comece já com a primeira leitura real, em vez de começar do zero

/*
Função que filtra a leitura bruta do ADC usando uma média móvel exponencial
(EMA), feita inteiramente em ponto fixo (sem usar float, que é lento no AVR
e exige biblioteca extra pro sprintf imprimir). O truque é manter um
acumulador "ampliado" por 2³ = 8: a cada chamada, a gente subtrai uma
fração (1/8) do acumulador e soma a leitura nova, o que equivale
matematicamente a um filtro exponencial com alpha = 1/8, só que usando
apenas somas, subtrações e deslocamentos de bits (>> e <<), que são
instruções de 1 ciclo no processador, em vez de multiplicação ou divisão
de verdade. Na primeira chamada, o acumulador é inicializado já com a
primeira leitura real (ampliada), pra não começar filtrando a partir de
zero e demorar vários segundos só pra "subir" até a temperatura real.
*/
uint16_t filtro_adc(uint16_t tNovaLeitura)
{
	if (!gFiltroInicializado)
	{
		gAcumuladorFiltro = (uint16_t)(tNovaLeitura << FILTRO_SHIFT);
		gFiltroInicializado = 1;
	}

	gAcumuladorFiltro = gAcumuladorFiltro - (gAcumuladorFiltro >> FILTRO_SHIFT) + tNovaLeitura;

	return (gAcumuladorFiltro >> FILTRO_SHIFT);
}

/*
Função que configura o Timer1 (o único timer de 16 bits do ATmega328P) em
modo CTC (Clear Timer on Compare Match), fazendo ele gerar uma interrupção
a cada 250 milissegundos. O bit WGM12 liga o modo CTC, o bit CS12 sozinho
seleciona o prescaler 256 (tabela de seleção de clock do datasheet), e o
valor de OCR1A é calculado assim: o timer conta a 16MHz/256 = 62500
contagens por segundo, então pra fechar 250ms (0,25s) precisamos de
62500 × 0,25 = 15625 contagens. Como o timer reseta depois de atingir
OCR1A, usamos OCR1A = 15624 (15625 - 1) pra ter exatamente 250ms, sem
erro de arredondamento. O OCIE1A habilita a interrupção de "compare
match A", que é disparada toda vez que o contador bate nesse valor.
*/
void timer1_init(void)
{
	TCCR1A = 0;
	TCCR1B = (1<<WGM12) | (1<<CS12); //modo CTC, prescaler = 256
	OCR1A = 15624; //(15624+1)*256/16MHz = 0.25s exatos
	TIMSK1 |= (1<<OCIE1A);
}

/*
Interrupção disparada pelo Timer1 toda vez que o contador bate em OCR1A,
ou seja, a cada 250ms certinhos, garantidos pelo hardware. Repare que
essa interrupção não faz leitura de ADC nem cálculo de controle nem nada
pesado - ela só levanta a flag gTick, que o loop principal fica observando.
Isso é proposital: interrupções devem ser curtas, senão elas atrasam ou
até bloqueiam outras interrupções importantes (como a do RX da serial)
enquanto estão rodando.
*/
ISR(TIMER1_COMPA_vect)
{
	gTick = 1;
}

//Protótipos das funções (declaradas aqui em cima pra poderem ser chamadas em qualquer ordem no arquivo)
void uart_init(uint32_t tBaud);
void uart_putchar(char tDado);
void uart_print(const char *tStr);
void processar_comando(void);
void salvar_historico_eeprom(void);
void enviar_historico_eeprom(void);

ISR(USART_RX_vect) //Interrup��o solicitada toda vez que um caractere chega no pino RX
{
	char tByte = UDR0; //L� o registrador onde o caractere recebido fica guardado e salva na vari�vel tempor�ria tByte

	if (tByte == '\n' || tByte == '\r') //Verifica se o caractere recebido foi uma quebra de linha (\n) ou um Enter (\r)
	{
		if (gUartIndex > 0) //Garante que o usu�rio digitou alguma coisa antes de apertar Enter (�ndice tem que ser maior que zero)
		{
			gUartBuffer[gUartIndex] = '\0'; //Adiciona o caractere nulo no final do buffer transformando o vetor em uma string v�lida no C
			gComandoPronto = 1; //Sinaliza para o programa principal que h� um comando completo esperando para ser processado
			gUartIndex = 0; //Reseta o �ndice para que o pr�ximo comando comece a ser gravado do in�cio do vetor
		}
	}
	/*Se o caractere n�o for um enter ele entra aqui, verifica se ainda h� espa�o no 
	buffer para evitar estouro de mem�ria (UART_BUFFER_SIZE - 1),se houver espa�o, 
	o caractere � salvo no buffer e o �ndice � incrementado (gUartIndex++)*/
	else if (gUartIndex < (UART_BUFFER_SIZE - 1))
	{
		gUartBuffer[gUartIndex++] = tByte;
	}
}

void uart_init(uint32_t tBaud)//Fun��o que configura a velocidade e os pinos da comunica��o serial
{
	uint16_t tUbrr = (F_CPU / (16UL * tBaud)) - 1; //Equa��o da tabela 19-1 do datasheet pro c�lculo da taxa de transmiss�o (UBRR)

	UBRR0H = (uint8_t)(tUbrr >> 8);
	UBRR0L = (uint8_t)tUbrr;

	UCSR0B = (1<<TXEN0) //Habilita transmiss�o
		   | (1<<RXEN0) //Habilita recep��o
		   | (1<<RXCIE0); //Habilita interrup��o de recep��o
		   
	UCSR0C = (1<<UCSZ01) | (1<<UCSZ00); //frame de 8 bits, sem paridade e 1 bit de parada
}

/*
Fun��o que envia um �nico caractere, o while fica travado esperando o bit UDRE0 
(do registrador UCSR0A) ficar em 1, o que significa que o hardware terminou de 
enviar o caractere anterior e o buffer de transmiss�o est� vazio, quando libera, 
ele joga o caractere em UDR0 para ser transmitido fisicamente
*/
void uart_putchar(char tDado)
{
	while (!(UCSR0A & (1<<UDRE0)));
	UDR0 = tDado;
}

/*
Fun��o que recebe um ponteiro para um texto (string) e vai enviando caractere 
por caractere usando a fun��o uart_putchar at� encontrar o fim do texto (\0).
*/
void uart_print(const char *tStr)
{
	while (*tStr)
	uart_putchar(*tStr++);
}

/*
Cria uma c�pia local (tComando) do buffer global da UART. Isso serve para liberar o 
buffer original de forma segura ou manipul�-lo sem interfer�ncias.
*/
void processar_comando(void)
{
	char tComando[UART_BUFFER_SIZE];
	
	//Copia o conte�do do buffer global para uma vari�vel local, evita que um novo dado chegue pela UART enquanto outro dado esteja sendo processado
	for (uint8_t i = 0; i < UART_BUFFER_SIZE; i++)
	tComando[i] = gUartBuffer[i];
	
	//Verifica se o texto enviado come�a exatamente com as letras "SP=" (Set Point).
	if (tComando[0]=='S' && tComando[1]=='P' && tComando[2]=='=')
	{
		uint16_t tNovoSetpoint = (uint16_t)atoi(&tComando[3]);

		if (tNovoSetpoint <= 110)
		{
			gSetpoint = tNovoSetpoint;
			uart_print("OK\r\n");
		}
		else
		{
			uart_print("ERRO: setpoint fora da faixa (0-110)\r\n");
		}
	}
	
	//Verifica se o texto enviado começa exatamente com as letras "AL=" (Alarme).
	else if (tComando[0]=='A' && tComando[1]=='L' && tComando[2]=='=')
	{
		uint16_t tNovoLimiar = (uint16_t)atoi(&tComando[3]);

		if (tNovoLimiar <= 110)
		{
			gLimiarAlarme = tNovoLimiar;
			uart_print("OK\r\n");
		}
		else
		{
			uart_print("ERRO: limiar fora da faixa (0-110)\r\n");
		}
	}
	
	//Verifica se o texto enviado é exatamente o comando "HIST" (Histórico).
	else if (tComando[0]=='H' && tComando[1]=='I' && tComando[2]=='S' && tComando[3]=='T')
	{
		enviar_historico_eeprom();
	}
	else
	{
		uart_print("ERRO: comando invalido\r\n");
	}
}

/*
Função que salva a temperatura atual na EEPROM em formato de buffer circular.
Só grava se a temperatura mudou desde a última gravação, pra preservar a vida
útil da EEPROM (limite de ~100mil ciclos de escrita por byte). Quando o índice
chega no fim do vetor, ele volta pro início (sobrescrevendo a leitura mais antiga).
*/
void salvar_historico_eeprom(void)
{
	if (gTemperatura != gUltimaTempGravada)
	{
		eeprom_update_byte(gEepromHistorico + gHistIndice, (uint8_t)gTemperatura);

		gHistIndice++;
		if (gHistIndice >= HISTORICO_TAMANHO)
		gHistIndice = 0;

		eeprom_update_byte(gEepromIndice, gHistIndice); //Salva o índice atualizado pra sobreviver a um reset

		gUltimaTempGravada = gTemperatura;
	}
}

/*
Função que lê o histórico salvo na EEPROM e manda pela serial, da amostra mais
antiga pra mais recente, na ordem correta (começando pela posição seguinte ao
índice atual, já que ali está a gravação mais antiga do buffer circular).
*/
void enviar_historico_eeprom(void)
{
	char tMsg[16];
	uart_print("HISTORICO:\r\n");

	for (uint8_t i = 0; i < HISTORICO_TAMANHO; i++)
	{
		uint8_t tPos = (gHistIndice + i) % HISTORICO_TAMANHO;
		uint8_t tValor = eeprom_read_byte(gEepromHistorico + tPos);

		sprintf(tMsg, "%u: %u C\r\n", i + 1, tValor);
		uart_print(tMsg);
	}
}

int main(void)
{
	DDRC &= ~((1<<DDC0) | (1<<DDC1)); //PC0 e PC1 como entrada
	PORTC |= (1<<PORTC0) | (1<<PORTC1); //Ativa pull-up do PC0 e PC1

	DDRD |= (1<<DDD5); //PD5 como sa�da (PWM que controla a potencia da lampada)
	DDRD |= (1<<DDD7); //PD7 como saída (aciona o buzzer do alarme sonoro)

	//Modo fast PWM
	TCCR0A = (1<<COM0B1) | (1<<WGM01) | (1<<WGM00);
	TCCR0B = (1<<WGM02) | (1<<CS01);

	OCR0A = 99;
	OCR0B = 0;

	ADMUX = (1<<REFS1)|(1<<REFS0) //Referencia de tens�o interna de 1,1V
	| (0<<MUX3)|(1<<MUX2)|(0<<MUX1)|(1<<MUX0); //ADC5

	ADCSRA = (1<<ADEN) //Habilita ADC
	| (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); //Prescaler do ADC em 128

	DIDR0 = (1<<ADC5D); //Desabilita buffer do ADC5 que j� est� sendo usado como entrada anal�gica

	uart_init(9600); //inicializa uart com 9600 de baud

	sei(); //habilita interrup��es globais
	
	gHistIndice = eeprom_read_byte(gEepromIndice); //Recupera o índice do histórico salvo na EEPROM (sobrevive a reset/desligamento)
	if (gHistIndice >= HISTORICO_TAMANHO) //Proteção: se a EEPROM nunca foi gravada (vem com 0xFF de fábrica), zera o índice
	gHistIndice = 0;

	while (1)
	{
		ADCSRA |= (1<<ADSC);
		while (ADCSRA & (1<<ADSC));
		
		//Converte valor bruto do ADC (0-1023) em �C
		gTemperatura = ((uint32_t)ADC * 1100) / 1024 / 10;
		
		//Salva a leitura atual no histórico da EEPROM (buffer circular de 10 posições)
		salvar_historico_eeprom();

		//Controle ON-OFF com histerese de +- 1�C
		if (!gLampadaLigada && gTemperatura <= (gSetpoint - 1))
		{
			gLampadaLigada = 1;
			OCR0B = 99;
		}

		if (gLampadaLigada && gTemperatura >= (gSetpoint + 1))
		{
			gLampadaLigada = 0;
			OCR0B = 0;
		}
		
		//Se o bot�o for pressionado decrementa SP
		if (!(PINC & (1<<PINC0)))
		{
			if (gSetpoint > 0)
			gSetpoint--;

			while (!(PINC & (1<<PINC0)));
			_delay_ms(100);
		}
		
		//Se o bot�o for pressionado aumenta SP
		if (!(PINC & (1<<PINC1)))
		{
			if (gSetpoint < 110)
			gSetpoint++;

			while (!(PINC & (1<<PINC1)));
			_delay_ms(100);
		}
		
		//Verifica se a temperatura passou do limiar de alarme (definido via serial com AL=) e dispara o aviso pela serial
		if (gTemperatura > gLimiarAlarme && !gAlarmeAtivo)
		{
			gAlarmeAtivo = 1;
			PORTD |= (1<<PORTD7); //Liga o buzzer
			uart_print("ALARME: TEMP_ALTA\r\n");
		}

		if (gTemperatura <= gLimiarAlarme && gAlarmeAtivo)
		{
			gAlarmeAtivo = 0;
			PORTD &= ~(1<<PORTD7); //Desliga o buzzer
			uart_print("ALARME: TEMP_NORMALIZADA\r\n");
		}

		//String com temperatura atual e setpoint enviada pela serial
		char tMsg[32];
		sprintf(tMsg, "TEMP=%u;SET=%u\r\n", gTemperatura, gSetpoint);
		uart_print(tMsg);

		//Se um comando chegar pela UART processa esse comando e zera para n�o ser processado novamente
		if (gComandoPronto)
		{
			gComandoPronto = 0;
			processar_comando();
		}

		_delay_ms(250);
	}
}