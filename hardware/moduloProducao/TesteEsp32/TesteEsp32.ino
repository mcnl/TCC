/*
  Usa um loop for para dados e imprime cada número em vários formatos.
*/
int x = 0;  // variable

void setup() {
  Serial.begin(9600); // abre a porta serial a 9600 bps:
}

void loop() {
  // imprime rótulos para cada base
  Serial.print("NUMERO FUNFOU"); // imprime um rótulo
  Serial.print("\t"); // imprime uma tabulação (TAB)

  Serial.print("DEC");
  Serial.print("\t");

  Serial.print("HEX");
  Serial.print("\t");

  Serial.print("OCT");
  Serial.print("\t");

  Serial.print("BIN");
  Serial.println("\t");

  for (x = 0; x < 64; x++) {  // apenas uma parte da tabela ASCII, edite para mais ou menos valores

    // imprime cada número em vários formatos:
    Serial.print(x);       // imprime como decimal codificado em ASCII- o mesmo que "DEC"
    Serial.print("\t");    // imprime uma tabulação

    Serial.print(x, DEC);  // imprime como decimal codificado em ASCII
    Serial.print("\t");    // imprime uma tabulação

    Serial.print(x, HEX);  // imprime como hexadecimal codificado em ASCII
    Serial.print("\t");    // imprime uma tabulação

    Serial.print(x, OCT);  // imprime como octal codificado em ASCII
    Serial.print("\t");    // imprime uma tabulação

    Serial.println(x, BIN);// imprime como binário codificado em ASCII
    //então adiciona o retorno (enter) com "println"

    delay(200);             // delay de 200 milissegundos
  }
  Serial.println();         // imprime outro retorno
}
