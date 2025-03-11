package frc.robot.util;

import java.util.function.Consumer;

public class LoggedTunableGainsBuilder  {
    private LoggedTunableNumber kP;
    private LoggedTunableNumber kI;
    private LoggedTunableNumber kD;
    private LoggedTunableNumber kS;
    private LoggedTunableNumber kG;
    private LoggedTunableNumber kV;
    private LoggedTunableNumber kA;

    private LoggedTunableNumber kMMV;
    private LoggedTunableNumber kMMA;
    private LoggedTunableNumber kMMJ;
    private LoggedTunableNumber kMMEV;
    private LoggedTunableNumber kMMEA;

    public LoggedTunableGainsBuilder(String key,double kP, double kI, double kD,
        double kS, double kG, double kV, double kA, double kMMV, double kMMA, double kMMJ, double kMMEV, double kMMEA) {
        this.kP = new LoggedTunableNumber(key + "kP", kP);
        this.kI = new LoggedTunableNumber(key + "kI", kI);
        this.kD = new LoggedTunableNumber(key + "kD", kD);
        this.kS = new LoggedTunableNumber(key + "kS", kS);
        this.kG = new LoggedTunableNumber(key + "kG", kG);
        this.kV = new LoggedTunableNumber(key + "kV", kV);
        this.kA = new LoggedTunableNumber(key + "kA", kA);
        this.kMMV = new LoggedTunableNumber(key + "kMMV", kMMV);
        this.kMMA = new LoggedTunableNumber(key + "kMMA", kMMA);
        this.kMMJ = new LoggedTunableNumber(key + "kMMJ", kMMJ);
        this.kMMEV = new LoggedTunableNumber(key + "kMMEV", kMMEV);
        this.kMMEA = new LoggedTunableNumber(key + "kMMEA", kMMEA);
    }

    public void ifGainsHaveChanged(Consumer<Gains> gainsConsumer) {
        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            gainsConsumer.accept(build());
        }, kP, kI, kD, kS, kG, kV, kA, kMMV, kMMA, kMMJ, kMMEV, kMMEA);
    }

    public Gains build() {
        return Gains.builder()
        .kP(kP.get())
        .kI(kI.get())
        .kD(kD.get())
        .kS(kS.get())
        .kG(kG.get())
        .kV(kV.get())
        .kA(kA.get())
        .kMMV(kMMV.get())
        .kMMA(kMMA.get())
        .kMMJ(kMMJ.get())
        .kMMEV(kMMEV.get())
        .kMMEA(kMMEA.get())
        .build();
    }
}