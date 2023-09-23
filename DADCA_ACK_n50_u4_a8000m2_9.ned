package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n50_u4_a8000m2_9
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor;p=1815.47,7953.94");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2146.72,7819.56");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2791.04,7842.67");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3383.17,7990.13");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3832.33,7984.66");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4182.92,7925.71");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4674.00,7973.27");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5399.75,7783.95");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5957.10,7791.75");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=450.35,7596.90");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=946.93,7642.62");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1456.81,7632.16");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4929.33,7533.99");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1261.09,7266.65");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1713.61,7346.88");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2507.57,7497.08");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2861.89,7498.42");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3187.61,7346.47");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3642.82,7341.54");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4259.23,7453.66");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4591.98,7291.89");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5651.07,7491.89");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=391.47,7190.46");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=770.43,7183.55");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2245.83,7198.29");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3907.71,7016.08");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5420.83,7089.63");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=365.87,6786.94");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1933.59,6795.76");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2456.16,6885.73");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2954.47,6889.50");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3593.32,6817.16");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4577.75,6851.03");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4954.19,6890.48");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5933.16,6872.37");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=918.89,6520.79");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1567.11,6711.87");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4217.18,6711.38");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=53.71,6301.63");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2275.03,6428.67");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2822.86,6283.07");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3203.59,6474.02");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3574.30,6457.28");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3932.61,6425.68");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4727.73,6502.01");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5220.73,6421.16");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5629.71,6293.35");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=480.17,6058.71");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1377.00,6244.24");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3336.08,6149.47");
    }

}

