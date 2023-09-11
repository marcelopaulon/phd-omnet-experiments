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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=1.92,89.75");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=49.94,17.68");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=141.85,25.91");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=155.17,202.81");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=64.96,204.40");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=107.81,279.86");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=186.66,85.46");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=256.34,89.52");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=324.47,87.27");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=362.11,169.39");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=453.64,152.96");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=520.56,138.21");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=586.10,69.33");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=585.42,158.63");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=580.90,254.35");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=656.00,285.91");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=686.68,232.89");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=701.44,363.38");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=748.66,296.86");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=773.19,359.49");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=781.75,232.05");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=607.91,379.72");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=591.40,466.21");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=521.61,472.59");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=490.19,528.79");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=488.50,607.16");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=554.14,540.18");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=607.58,589.24");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=666.02,532.04");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=737.51,481.02");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=825.74,427.45");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=910.58,491.56");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=908.88,590.15");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=851.78,571.13");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=803.65,515.81");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=854.23,689.82");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=885.36,749.12");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=928.82,799.60");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=952.76,713.51");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=759.51,663.56");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=682.75,708.70");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=615.37,697.64");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=646.25,835.92");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=702.69,904.01");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=643.34,943.53");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=646.75,1016.60");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=672.90,1110.35");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=718.16,1049.53");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=735.99,977.78");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=792.10,875.97");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=827.86,821.64");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=549.73,877.13");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=462.64,859.36");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=474.92,794.30");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=438.90,914.86");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=548.04,962.70");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=488.46,1010.47");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=414.80,1076.78");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=501.53,1072.39");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=549.44,806.67");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=378.64,860.38");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=375.61,933.50");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=559.14,725.40");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=500.03,669.30");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=384.78,593.83");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=364.21,520.11");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=438.13,481.90");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=409.04,425.84");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=528.56,392.94");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=462.37,347.96");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=400.49,295.47");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=337.84,250.67");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=276.75,179.35");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=274.40,29.49");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=480.96,217.92");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=478.94,54.80");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=401.93,99.80");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=644.71,125.50");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=769.12,152.13");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=884.66,198.56");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=829.72,275.31");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=894.00,287.33");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=913.87,344.46");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=898.59,424.55");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=862.12,375.04");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=707.07,163.19");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=515.24,312.84");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=313.00,1046.17");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=346.55,1118.38");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=517.68,1196.28");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=531.48,1256.70");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=614.18,1270.21");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=560.14,1365.83");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=581.53,1130.92");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=652.22,1320.32");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=457.08,1123.66");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=651.28,1214.86");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=445.40,1261.79");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=501.49,1351.88");
    }

}

