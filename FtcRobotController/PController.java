
/*

Setpoint – Representa o valor desejado que o
sistema tenta alcançar. No caso de um robô, pode ser a
velocidade, posição ou orientação que ele precisa atingir.

Input (mínimo e máximo) – Refere-se ao intervalo de valores que o sistema pode observar.
Por exemplo, uma posição mínima e máxima de um motor.

Output (mínimo e máximo) – Define os limites mínimos e máximos da resposta do sistema,
como a potência aplicada a um motor.

Kp – Constante de proporcionalidade que controla a intensidade da reação do sistema ao erro.
Um valor maior resulta em ajustes mais rápidos, mas pode causar oscilações.

Erro – Calculado como a diferença entre o setPoint e o valor atual do sistema (input).
O controlador usa esse erro para ajustar a saída.

*/

public class PController {

    public double setPoint = 0, minInput = 0, maxInput = 0, minOutput = 0;
    public double maxOutput = 0, theresholdPercent = 0;
    private double currentError = 0;

    private double Kp;
    public PController (double Kp) {this.Kp = Kp;}

    public void setTheresholdPercent (double theresholdPercent){
        this.theresholdPercent = theresholdPercent; // A margem de erro aceitável, representada como uma porcentagem.
    }
    public void setSetPoint (double setPoint){
        this.setPoint = setPoint; //Define o valor ideal que o sistema tentará alcançar.
    }
    public void setInputRange(double minInput, double maxInput){
        this.minInput = Math.abs(minInput);
        this.maxInput = Math.abs(maxInput);
        /*Define os limites da entrada. Isso permite normalizar o cálculo,
        garantindo que os valores sejam tratados de forma consistente.*/
    }
    public void setOutputRange(double minOutput, double maxOutput){
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;

        /*Define os valores mínimo e máximo que o controlador pode produzir como saída.*/
    }
    public double getComputedOutput(double input){
        currentError = Math.abs(Math.abs(setPoint) - Math.abs(input));
        double computedOutput = currentError * Kp * (maxOutput - minOutput);
        if (computedOutput > (maxOutput - minOutput)){
            computedOutput = (maxOutput - minOutput);
        }
        return computedOutput;

        /*Calcula a saída proporcional ao erro atual:
        O erro é calculado como a diferença entre o setPoint e o valor de entrada (input).

        A saída é proporcional ao erro, multiplicada por Kp e escalada pelo intervalo de saída (maxOutput - minOutput).

        Limitação de saída: Se o valor calculado exceder o limite máximo, ele é ajustado para o máximo permitido.
        */
    }
    public boolean hasPControllerReachedTarget(){
        double percentDifferenceFromTarget = (currentError / (maxInput - minInput)) * 100;
        if (percentDifferenceFromTarget < theresholdPercent){
            return true;
        }
        return false;

        /*Verifica se o sistema alcançou o objetivo dentro do limite aceitável de erro:

        Calcula a diferença percentual entre o erro e o intervalo de entrada (minInput a maxInput).

        Retorna true se o erro for menor que o limite definido por theresholdPercent*/

    }

}

