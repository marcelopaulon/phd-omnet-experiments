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

network DADCA_ACK_n100_a8000m2_lpnFn1_0
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
        @display("i=misc/sensor,p=24.46,112.30");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=104.79,159.87");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=127.14,29.94");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=70.26,257.13");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=87.53,335.66");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=105.74,427.16");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=51.53,459.65");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=117.86,563.93");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=202.39,514.08");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=239.26,452.39");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=287.06,409.78");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=350.00,464.18");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=405.03,508.59");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=402.94,406.57");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=471.03,366.97");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=531.98,406.01");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=603.25,376.26");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=684.26,343.58");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=629.57,304.36");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=795.70,274.89");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=753.32,359.09");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=711.70,416.61");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=772.95,429.37");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=837.28,478.29");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=807.52,534.33");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=894.73,574.51");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=914.05,650.85");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=994.09,551.34");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=1077.93,508.02");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=1160.55,465.09");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=1209.75,405.10");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=1153.77,354.59");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=1087.43,329.89");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=1022.21,290.81");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=1067.93,224.30");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=925.82,353.12");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=855.37,392.21");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=940.45,288.57");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=896.61,207.83");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=861.33,322.14");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=783.45,138.86");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=732.97,192.40");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=886.74,138.17");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=956.69,100.06");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=947.11,6.93");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=893.82,66.28");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=809.56,61.20");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=692.43,110.38");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=634.86,135.12");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=653.64,21.49");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=604.38,67.07");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=531.18,81.81");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=527.79,20.60");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=594.86,7.23");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=716.67,17.23");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=746.73,83.64");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=836.73,203.68");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=1057.54,94.42");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=1111.69,148.09");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=1019.96,3.96");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=1028.38,178.90");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=955.21,226.61");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=1330.53,375.80");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=1282.30,297.73");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=1196.69,310.51");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=1145.18,219.00");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=1011.45,356.42");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=715.58,281.87");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=949.14,435.26");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=1001.96,644.22");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=929.96,711.66");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=813.71,711.68");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=817.11,651.60");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=763.87,622.31");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=1026.19,769.02");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=950.84,775.19");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=1072.56,657.32");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=1090.53,573.08");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=1127.54,721.74");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=1205.02,807.19");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=1268.31,841.02");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=1277.74,900.98");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=1194.59,937.07");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=1175.65,872.18");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=1080.36,879.59");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=1002.80,925.85");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=953.03,973.98");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=1030.26,1035.38");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=932.39,1047.01");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=1115.12,1067.98");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=1144.48,1157.69");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=1081.50,1141.06");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=1137.22,1240.64");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=1054.69,1205.19");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=986.46,1236.97");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=993.96,1164.62");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1050.44,1268.67");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=1002.75,1098.72");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=950.82,890.81");
    }

}

