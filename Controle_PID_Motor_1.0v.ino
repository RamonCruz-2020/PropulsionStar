#include <HX711.h>
#include <PID_v1.h>

#define PIN_LOX_INPUT 2         //Entrada do Sensor de Fluxo na porta digital 2 da linha de Oxigenio Líquido.
#define PIN_LOX_OUTPUT 3        //Saida para valvula na porta PWM 3 da linha de Oxigenio Líquido.
#define PIN_ETHANOL_INPUT 4     //Entrada do Sensor de Fluxo na porta digital 4 da linha de Etanol.
#define PIN_ETHANOL_OUTPUT 5    //Saida para valvula na porta PWM 5 da linha de Etanol.
#define PIN_DT 6                //Entrada DT do módulo HX711 da Celula de Carga.
#define PIN_SCK 7               //Entrada SCK do módulo HX711 da Celula de Carga.

#define WeightMin 0.010         //Peso minino da Celula de Carga.
#define WeightMax 100.0         //Peso maximo da Celula de Carga.

#define scale_ 0.0f             //Valor da escala para a calibração do modulo HX711.

unsigned long lastSend = 0;

float weight = 0;               //Armazenar valor de medida em Kg/f

int LoxCountPulse,              //Contador de pulso do sensor de fluxo da linha de Oxigenio Líquido
    EthanolCountPulse;          //Contador de pulso do sensor de fluxo da linha de Etanol.

//--------linha de Oxigenio Líquido-------
double LoxSetPoint,           //SetPoint do controle PID.
       LoxInput,              //A entrada do controle PID.
       LoxOutput,             //A saida do controle PID.
       LoxKp = 0.15,          //O Proporcional do PID.
       LoxKi = 0.8,           //O Integral do PID.
       LoxKd = 0.0,           //A Derivativo do PID.
       LoxDensity = 1141,     //Densidade: Lox: (-183°C) 1,141g/cm3.
       LoxL_Min,              //O Fluxo do Sensor em L/Min.
       Loxg_s;                //O Fluxo do Sensor convertido em g/s.
       
//--------linha de Etanol-------
double EthanolSetPoint,       //SetPoint do controle PID.
       EthanolInput,          //A entrada do controle PID.
       EthanolOutput,         //A saida do controle PID.
       EthanolKp = 0.15,      //O Proporcional do PID.
       EthanolKi = 0.8,       //O Integral do PID.
       EthanolKd = 0.0,       //A Derivativo do PID.
       EthanolDensity = 789,  //Densidade: Etanol: (20°C) 0,789g/cm3.
       EthanolL_Min,          //O Fluxo do Sensor em L/Min.
       Ethanolg_s;            //O Fluxo do Sensor convertido em g/s.

//--------Sistema de Proporção & Quantidade dos Reagentes​-------
double SpqrSetPoint,          //SetPoint do controle PID.
       SpqrInput,             //A entrada do controle PID.
       SpqrOutput,            //A saida do controle PID.
       SpqrKp = 0.15,         //O Proporcional do PID.
       SpqrKi = 0.8,          //O Integral do PID.
       SpqrKd = 0.0,          //A Derivativo do PID.
       SpqrRateMaxg_s,        //Taxa maxima de vazão dos Reagentes em g/s.
       SpqrProportion;        //Valor de Proporção dos Reagentes.

HX711 scale;

PID LoxPID(&LoxInput, &LoxOutput, &LoxSetPoint, LoxKp, LoxKi, LoxKd, DIRECT);                                   //Instaciando a classe PID da linha de Oxigenio Líquido.

PID EthanolPID(&EthanolInput, &EthanolOutput, &EthanolSetPoint, EthanolKp, EthanolKi, EthanolKd, DIRECT);       //Instaciando a classe PID da linha de Etanol.

PID SpqrPID(&SpqrInput, &SpqrOutput, &SpqrSetPoint, SpqrKp, SpqrKi, SpqrKd, DIRECT);                            //Instaciando a classe PID do Sistema de Proporção & Quantidade dos Reagentes​.

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);                         //Configurando a velocidade de comunicação da porta serial.

  pinMode(PIN_LOX_INPUT, INPUT);
  attachInterrupt(0, loxincpulse, RISING);      //Configura o pino PIN_LOX_INPUT(Interrupção 0) para trabalhar como interrupção.

  pinMode(PIN_ETHANOL_INPUT, INPUT);
  attachInterrupt(0, ethanolincpulse, RISING);  //Configura o pino PIN_ETHANOL_INPUT(Interrupção 0) para trabalhar como interrupção.

  scale.begin(PIN_DT, PIN_SCK);     //CONFIGURANDO OS PINOS DA BALANÇA
  scale.set_scale(scale_);          //ENVIANDO O VALOR DA ESCALA CALIBRADO

  delay(2000);
  scale.tare();                     //ZERANDO A BALANÇA PARA DESCONSIDERAR A MASSA DA ESTRUTURA

  SpqrRateMaxg_s = 2000;            //Definir o valor de taxa maxima de vazão dos Reagentes em g/s.
  SpqrProportion = 1.47;            //Definir o valor de Proporção dos Reagentes.
  SpqrSetPoint = 10;                //Definir o SetPoint em Kg/f.

  LoxPID.SetMode(AUTOMATIC);        //Habilitando o controle PID da linha de Oxigenio Líquido.
  LoxPID.SetSampleTime(50);         //Configurando a taxa de execução do controle PID da linha de Oxigenio Líquido.

  EthanolPID.SetMode(AUTOMATIC);    //Habilitando o controle PID da linha de Etanol.
  EthanolPID.SetSampleTime(50);     //Configurando a taxa de execução do controle PID da Etanol.

  SpqrPID.SetMode(AUTOMATIC);       //Habilitando o controle PID do Sistema de Proporção & Quantidade dos Reagentes​.
  SpqrPID.SetSampleTime(50);        //Configurando a taxa de execução do controle PID do Sistema de Proporção & Quantidade dos Reagentes​.

}

void loop() {
  // put your main code here, to run repeatedly:

  LoxCountPulse = 0;                                    //Zera a variável para contar os giros por segundos
  EthanolCountPulse = 0;
  sei();                                                //Habilita interrupção
  delay (1000);                                         //Aguarda 1 segundo
  cli();                                                //Desabilita interrupção
  
  LoxL_Min = LoxCountPulse / 5.5;                       //Converte para L/min
  EthanolL_Min = EthanolCountPulse / 5.5;               //Converte para L/min

  scale.power_up();                                     //LIGANDO O SENSOR
    
  weight = scale.get_units(5);                          //SALVANDO NA VARIAVEL O VALOR DA MÉDIA DE 5 MEDIDAS
    
  if (weight <= WeightMin ){                            //CONFERE SE A MASSA ESTÁ NA FAIXA VÁLIDA
    scale.tare();                                       //ZERA A BALANÇA CASO A MASSA SEJA MENOR QUE O VALOR MIN
    weight = 0;
    //Serial.println("Tara Configurada!");
  } else if ( weight >= WeightMax ){
    scale.tare();                                       //ZERA A BALANÇA CASO A MASSA SEJA MAIOR QUE O VALOR MÁX
    weight = 0;
    //Serial.println("Tara Configurada!");
  } else {
    //Serial.println(weight,3);
  }

  scale.power_down();                                   //DESLIGANDO O SENSOR

  //LoxInput = analogRead(PIN_LOX_INPUT);
  //--------linha de Oxigenio Líquido-------
  Loxg_s = (1000/LoxDensity)*LoxL_Min;                  //Convertendo L/min em g/s.
  LoxInput = Loxg_s;                                    //Passando o valor de g/s para entrada do controle PID.
  LoxPID.Compute();                                     //Executando o controle PID.
  analogWrite(PIN_LOX_OUTPUT, LoxOutput);               //A saida do controle PID para o Servo Motor da valvula

  //--------linha de Etanol-------
  Ethanolg_s = (1000/EthanolDensity)*EthanolL_Min;      //Convertendo L/min em g/s.
  EthanolInput = Ethanolg_s;                            //Passando o valor de g/s para entrada do controle PID.
  EthanolPID.Compute();                                 //Executando o controle PID.
  analogWrite(PIN_ETHANOL_OUTPUT, EthanolOutput);       //A saida do controle PID para o Servo Motor da valvula

  //--------Sistema de Proporção & Quantidade dos Reagentes-------
  SpqrInput = weight;                                                         //Passando o valor em Kg/f para entrada do controle PID.
  SpqrPID.Compute();                                                          //Executando o controle PID.
  LoxSetPoint = (SpqrRateMaxg_s/255)*SpqrOutput;                              //Configurando o SetPoint em g/s do controle PID da linha de Oxigenio Líquido.
  EthanolSetPoint = ((SpqrRateMaxg_s/255)*SpqrOutput)/SpqrProportion;         //Configurando o SetPoint em g/s do controle PID da linha de Etanol.

  if(millis() - lastSend > 100){
    lastSend = millis();

    Serial.print(LoxSetPoint);
    Serial.print(" ");
    Serial.print(LoxInput);
    Serial.print(" ");
    Serial.print(LoxOutput);
    Serial.print(" ");
    Serial.print(EthanolSetPoint);
    Serial.print(" ");
    Serial.print(EthanolInput);
    Serial.print(" ");
    Serial.print(EthanolOutput);
    Serial.print(" ");
    Serial.print(SpqrSetPoint);
    Serial.print(" ");
    Serial.print(SpqrInput);
    Serial.print(" ");
    Serial.print(SpqrOutput);
    Serial.println(" ");
    }
}

void loxincpulse ()
{ 
  LoxCountPulse++; //Incrementa a variável de contagem dos pulsos
} 

void ethanolincpulse ()
{ 
  EthanolCountPulse++; //Incrementa a variável de contagem dos pulsos
}
