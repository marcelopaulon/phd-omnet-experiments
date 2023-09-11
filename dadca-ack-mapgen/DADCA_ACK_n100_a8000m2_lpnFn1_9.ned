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

network DADCA_ACK_n100_a8000m2_lpnFn1_9
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
        @display("i=misc/sensor;p=45.48,182.94");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=59.23,260.95");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=124.37,222.29");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=189.42,262.49");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=210.95,177.70");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=280.63,109.35");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=219.49,107.27");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=219.87,37.43");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=276.81,13.31");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=344.04,57.54");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=276.94,197.89");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=109.00,304.31");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=30.79,342.74");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=97.64,389.00");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=39.29,407.35");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=180.00,378.64");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=149.37,445.35");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=129.14,527.51");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=63.45,553.70");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=53.86,475.62");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=173.08,594.24");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=243.47,612.59");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=336.15,581.66");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=389.40,522.10");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=393.16,449.98");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=296.95,471.88");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=281.33,401.42");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=366.21,712.66");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=389.02,772.94");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=311.84,799.30");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=265.23,753.45");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=192.68,821.43");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=254.59,845.11");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=348.70,864.49");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=422.22,834.31");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=451.47,742.03");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=480.08,685.86");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=403.36,656.93");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=489.34,869.32");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=568.97,836.92");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=620.80,791.43");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=586.94,741.50");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=610.74,671.70");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=524.06,636.03");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=654.62,571.63");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=563.52,565.92");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=518.93,520.19");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=522.87,454.76");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=587.98,429.64");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=614.82,370.58");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=556.47,292.92");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=477.83,295.34");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=454.90,202.73");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=507.10,101.48");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=561.68,17.69");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=617.35,53.17");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=672.65,142.12");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=692.84,10.43");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=766.92,39.08");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=792.19,103.08");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=755.60,155.12");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=633.62,191.87");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=579.55,139.87");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=645.40,265.73");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=752.66,247.09");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=774.13,314.41");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=679.81,466.68");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=782.59,582.25");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=823.14,506.40");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=875.32,453.88");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=825.54,405.20");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=901.09,393.55");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=985.65,441.57");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=952.33,502.27");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=945.23,569.33");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=962.99,632.26");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1017.78,697.94");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1079.27,648.68");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1171.95,612.71");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1178.65,688.18");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1264.30,681.15");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1189.70,756.26");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1168.51,853.17");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1125.06,787.11");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1041.66,919.53");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1116.82,910.27");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1204.26,933.44");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1274.65,824.62");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1025.54,605.82");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1084.59,518.31");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1147.88,540.70");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1074.44,418.18");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1088.79,324.03");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1136.74,399.55");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1023.11,495.77");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=525.46,357.63");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=859.11,144.19");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=862.28,76.08");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=867.05,15.49");
    }

}

