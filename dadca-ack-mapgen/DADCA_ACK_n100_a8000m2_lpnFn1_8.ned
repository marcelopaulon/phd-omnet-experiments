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
        @display("i=misc/sensor,p=140.35,32.93");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=216.75,7.34");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=222.89,151.83");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=311.08,193.48");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=388.39,178.99");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=418.16,118.74");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=530.24,90.95");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=490.50,143.42");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=550.28,156.96");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=485.29,48.80");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=431.92,15.20");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=344.54,53.20");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=284.60,30.23");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=179.77,92.34");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=65.61,4.88");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=80.01,161.71");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=155.77,188.84");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=13.88,85.15");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=36.21,255.72");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=113.04,303.58");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=66.55,384.13");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=83.41,445.41");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=19.40,448.45");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=28.03,521.71");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=8.81,602.60");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=99.32,592.44");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=109.63,531.44");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=188.05,582.83");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=157.89,648.87");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=69.96,661.39");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=71.78,753.14");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=309.05,555.62");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=332.80,475.28");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=346.98,396.55");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=255.18,369.86");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=166.52,497.69");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=174.25,405.25");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=197.03,277.68");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=442.27,209.19");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=503.37,278.43");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=406.10,261.66");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=333.11,131.72");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=242.36,85.97");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=563.82,252.02");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=595.91,23.16");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=604.99,123.41");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=671.48,135.67");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=647.10,59.78");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=792.61,56.51");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=760.79,114.44");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=782.51,235.84");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=657.94,203.17");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=725.13,178.61");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=635.95,305.95");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=720.81,282.94");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=841.73,186.94");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=854.22,255.40");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=981.04,213.44");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=1008.79,128.24");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=960.80,85.24");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=1057.56,274.33");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=1000.30,326.64");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=936.20,330.70");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=892.64,421.53");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=831.26,416.79");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=824.07,317.87");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=943.03,155.98");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=1060.38,11.65");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=1128.89,8.42");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=1197.78,24.18");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=1140.62,80.82");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=1075.56,90.70");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=1242.96,85.22");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=1279.56,26.92");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=1213.86,156.17");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=1295.61,135.26");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=1362.59,130.49");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=1059.01,166.28");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=881.37,41.39");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=919.56,220.77");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=881.33,113.56");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=1106.06,228.38");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=1126.38,139.95");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=1111.30,306.41");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=1189.66,342.11");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=1280.05,381.07");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=1332.46,302.12");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=1376.00,352.72");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=1236.50,427.01");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=1313.11,441.62");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=1235.63,494.21");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=1181.73,530.80");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=1096.27,458.65");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=1101.50,378.89");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=1166.27,416.60");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=1169.61,211.74");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1279.57,193.72");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=1227.75,278.56");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=1347.68,215.38");
    }

}

