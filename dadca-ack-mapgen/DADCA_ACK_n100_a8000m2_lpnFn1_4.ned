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

network DADCA_ACK_n100_a8000m2_lpnFn1_4
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
        @display("i=misc/sensor,p=87.97,26.45");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=26.61,74.39");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=16.29,172.92");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=185.14,75.50");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=249.62,98.40");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=352.99,178.00");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=287.34,175.88");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=222.50,202.97");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=191.23,275.43");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=224.14,403.45");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=135.63,385.78");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=274.60,325.98");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=336.17,298.39");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=366.29,363.78");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=102.43,262.89");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=142.05,190.61");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=74.12,380.77");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=15.24,419.65");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=80.57,495.00");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=16.98,560.99");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=69.43,597.67");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=119.96,680.95");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=201.58,708.40");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=259.95,757.40");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=313.58,674.41");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=371.53,747.91");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=348.44,840.88");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=344.28,595.50");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=407.48,529.99");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=415.26,613.31");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=225.99,821.30");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=292.24,811.88");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=101.99,766.63");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=128.35,885.99");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=61.96,942.26");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=91.39,1011.98");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=95.52,1101.54");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=33.59,815.55");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=298.48,490.43");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=380.25,445.31");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=447.25,470.20");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=536.78,470.65");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=551.67,389.33");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=541.70,308.85");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=484.68,330.42");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=629.81,465.87");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=583.74,532.11");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=642.00,568.95");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=673.79,512.04");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=746.46,456.22");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=678.45,401.51");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=653.21,335.82");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=826.65,374.94");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=714.43,347.41");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=818.18,470.20");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=872.29,441.14");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=938.91,431.07");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=922.20,308.12");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=963.95,494.20");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=973.27,554.36");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=1034.73,615.58");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=1047.39,696.26");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=1004.29,747.36");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=930.49,723.42");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=874.41,745.72");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=809.10,729.49");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=718.79,697.86");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=886.01,829.54");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=1101.57,776.50");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=1043.39,835.56");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=964.88,852.66");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=1107.83,846.07");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=1204.25,862.48");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=1246.97,801.76");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=1161.09,760.76");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=991.08,929.44");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=1015.42,1024.89");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=1050.71,965.05");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=1121.36,946.11");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=1002.41,1137.42");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=932.27,988.38");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=917.50,926.34");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=927.27,785.52");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=1200.95,714.63");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=1233.02,626.95");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=1179.73,582.89");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=1091.64,565.89");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=1123.23,637.21");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=1151.37,518.81");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=831.58,921.74");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=758.90,850.85");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=700.84,880.36");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=636.05,877.54");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=597.12,824.14");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=535.55,938.25");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=591.30,963.76");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=646.94,992.03");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=1191.15,995.95");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=1289.57,984.11");
    }

}

