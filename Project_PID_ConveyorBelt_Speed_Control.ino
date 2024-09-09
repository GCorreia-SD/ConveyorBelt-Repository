/*
Universidade Federal da Bahia (UFBA)
Departamento de Engenharia Elétrica e da Computação (DEEC) - Escola Politécnica
Alunos: Gabriel Correia e Mateus Fiscina
Disciplina: Laboratório Integrado VI
Professora: Cristiane Paim

MODELO DE CONTROLE DE VELOCIDADE DE ESTEIRA UTILIZANDO PID
*/

// ---------------- DEFINIÇÃO E CONFIGURAÇÕES INICIAIS ----------------
// Importação das Bibliotecas do Código (importante baixar antes de executar)
#include <AFMotor.h> // Biblioteca para controlar motores DC usando o Adafruit Motor Shield
#include <PID_v1.h>  // Biblioteca para controle PID
#include <Wire.h>    // Biblioteca para comunicação I2C
#include <LiquidCrystal_I2C.h> // Biblioteca para controlar LCD I2C

// Configuração do Motor
AF_DCMotor motor(1); // Cria um objeto motor na porta M1 do Adafruit Motor Shield

// Constantes do Controle PID
#define MIN_PWM 0   // Valor mínimo de PWM
#define MAX_PWM 255 // Valor máximo de PWM
#define KP 0.1625  // Constante proporcional do PID
#define KI 1.776      // Constante integral do PID
#define KD 0   // Constante derivativa do PID

// Variáveis do Sensor Infravermelho e PID
double rpm;                  // Armazena o valor atual de RPM do motor
volatile byte pulsos;        // Contador de pulsos do sensor infravermelho
unsigned long timeold;       // Marca o tempo da última atualização de RPM
int pinoSensor = 18;         // Pino do Arduino ligado ao pino D0 do sensor
unsigned int pulsosDisco = 20; // Número de pulsos por rotação completa do disco encoder
double velocidade = 0;         // Armazena o valor de saída do PID (PWM)
double velocidadeSetpoint = 100; // Velocidade desejada (setpoint) em RPM

// Constante para o pino do potenciômetro
const int potPin = A10; // Pino analógico onde o potenciômetro está conectado
const int switchPin = 30; // Pino digital onde o switch está conectado

// Substitua 0x27 pelo endereço do seu LCD encontrado pelo scanner I2C
LiquidCrystal_I2C lcd(0x27, 16, 2); // Inicializa o display LCD I2C com endereço 0x27, 16 colunas e 2 linhas

// Cria PID para controle
PID motorPID(&rpm, &velocidade, &velocidadeSetpoint, KP, KI, KD, DIRECT);


// ---------------- PROGRAMA A SER RODADO ----------------
// Função executada a cada interrupção
void contador() {
  pulsos++;  // Incrementa o contador de pulsos
}

// SETUP
void setup() {
  // Inicia Serial
  Serial.begin(9600); // Inicia a comunicação serial com velocidade de 9600 bps

  // Inicia o LCD I2C
  lcd.init(); // Inicializa o display LCD
  // lcd.begin(16, 2);
  lcd.backlight(); // Liga a luz de fundo do LCD
 
  // Configura Interrupção
  pinMode(pinoSensor, INPUT); // Define o pino do sensor como entrada
  pinMode(switchPin, INPUT); // Configura o pino do switch como entrada com pull-up interno

  attachInterrupt(digitalPinToInterrupt(pinoSensor), contador, FALLING); // Configura interrupção para contar pulsos em borda de descida
  pulsos = 0; // Inicializa o contador de pulsos
  rpm = 0;    // Inicializa o valor de RPM
  timeold = 0; // Inicializa o tempo antigo

  // Configura controle PID
  motorPID.SetOutputLimits(MIN_PWM, MAX_PWM); // Define os limites de saída do PID
  motorPID.SetMode(AUTOMATIC); // Define o modo do PID para automático
}


// LOOP
void loop() {
  // Lê o valor do potenciômetro e ajusta o setpoint de velocidade
  int potValue = analogRead(potPin); // Lê o valor do potenciômetro
  velocidadeSetpoint = map(potValue, 0, 1023, 0, 500); // Mapeia o valor do potenciômetro para a faixa desejada de RPM

  // Lê o estado do switch
  bool switchState = digitalRead(switchPin); // Lê o estado do switch (HIGH se pressionado, LOW se não pressionado)

  // Calcula RPM a cada 1 Segundo
  if (millis() - timeold >= 1000) { // Se um segundo se passou desde a última medição
    detachInterrupt(digitalPinToInterrupt(pinoSensor)); // Desabilita a interrupção durante o cálculo para evitar inconsistências
    rpm = (60 * 1000 / pulsosDisco) / (millis() - timeold) * pulsos; // Calcula RPM baseado nos pulsos contados
    timeold = millis(); // Atualiza o tempo antigo para o tempo atual
    pulsos = 0; // Reseta o contador de pulsos
    
    // Exibe TODOS os valores no serial monitor
    Serial.print("Setpoint: ");
    Serial.print(velocidadeSetpoint); // Exibe o setpoint de velocidade atual
    Serial.print("  ");
    Serial.print("Vel - saída PID: ");
    Serial.print(velocidade, 2); // Exibe a velocidade calculada pelo PID com duas casas decimais
    Serial.print("  ");
    Serial.print("RPM: ");
    Serial.println(rpm, 0); // Exibe o valor de RPM calculado
    Serial.print("Erro: ");
    Serial.println(velocidadeSetpoint - rpm); // Exibe o valor do erro do SetPoint em relação ao RPM medido pelo sensor

    // Habilita novamente a interrupção
    attachInterrupt(digitalPinToInterrupt(pinoSensor), contador, FALLING);
  }

  // Calcula o PWM do motor conforme Controle PID
  motorPID.Compute(); // Calcula o valor de saída do PID baseado no RPM atual e no setpoint


    // Lê o estado do switch
  //int switchState = digitalRead(switchPin);

  // Ajusta PWM no motor com base no estado do switch
  if (switchState == LOW) {
    motor.run(FORWARD); // Se o switch estiver pressionado, roda o motor para frente
   
  } else {
    motor.run(BACKWARD); // Caso contrário, roda o motor para trás
    
  }
  
  motor.setSpeed(velocidade);   // Define o PWM do motor com o valor calculado pelo PID


  // Apresenta os parâmetros no visor do LCD I2C
  lcd.clear(); // Limpa o display LCD
  lcd.setCursor(0, 0); // Move o cursor para a primeira linha
  lcd.print("Setpoint: "); // Exibe "Setpoint:" no LCD
  lcd.print(velocidadeSetpoint); // Exibe o setpoint de velocidade atual
  lcd.setCursor(0, 1); // Move o cursor para a segunda linha
  lcd.print("Snsr: "); // Exibe "Sensor:" no LCD
  lcd.print(rpm);      // Exibe o valor de RPM calculado
  lcd.print(" RPM");   // Exibe " RPM" após o valor de RPM
  //delay(100); // Aguarda 100 milissegundos antes de repetir o loop
}
