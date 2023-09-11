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

network DADCA_ACK_n100_a8000m2_lpnFn1_8
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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=100.91,173.42");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=192.44,138.76");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=261.85,96.29");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=318.19,65.65");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=393.14,21.43");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=445.74,59.20");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=496.29,135.56");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=538.92,74.64");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=412.00,137.87");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=357.32,218.82");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=366.20,278.34");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=259.36,227.34");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=278.55,285.12");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=325.73,366.31");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=446.68,323.68");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=477.63,250.87");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=510.00,336.25");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=551.32,397.69");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=563.19,259.96");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=636.10,216.19");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=643.16,303.05");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=707.00,269.31");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=782.71,251.50");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=821.04,331.84");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=853.51,238.39");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=813.78,151.86");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=878.64,100.57");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=710.75,137.64");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=625.58,107.97");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=652.00,12.63");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=732.91,57.14");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=148.88,55.02");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=35.05,80.56");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=98.86,14.62");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=44.87,147.66");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=76.26,240.26");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=170.18,254.10");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=80.49,346.63");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=44.47,438.89");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=127.98,450.96");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=7.73,324.47");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=200.92,382.74");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=202.27,467.94");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=203.34,529.95");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=151.71,582.32");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=191.90,632.03");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=301.52,554.73");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=279.24,487.55");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=346.90,424.22");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=138.66,319.66");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=7.19,521.80");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=66.92,539.98");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=41.65,598.47");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=130.20,386.90");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=280.90,612.83");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=289.66,691.59");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=374.79,577.93");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=358.95,651.30");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=448.48,603.62");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=506.28,627.16");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=588.82,589.44");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=631.83,632.83");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=654.66,726.20");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=594.36,745.95");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=565.23,814.02");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=701.31,860.18");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=760.82,876.74");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=795.64,944.60");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=711.97,947.64");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=623.14,941.09");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=530.36,870.40");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=483.87,784.16");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=478.50,947.98");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=461.23,889.51");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=671.51,1021.80");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=674.49,1111.66");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=756.58,1090.68");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=757.62,1154.99");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=784.21,1231.11");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=729.30,1285.90");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=824.46,1285.88");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=860.57,1209.83");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=606.43,1101.05");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=577.26,1020.44");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=511.59,1093.27");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=492.06,1035.88");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=429.18,1110.56");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=472.97,1177.76");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=548.10,959.56");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=385.80,947.28");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=310.48,888.71");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=265.00,840.26");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=220.92,780.35");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=157.87,803.16");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=108.76,730.96");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=146.20,918.03");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=119.73,851.34");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=68.58,792.51");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=43.23,847.35");
    }

}

