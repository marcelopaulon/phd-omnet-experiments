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

network DADCA_ACK_n100_a8000m2_lpnFn1_1
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
        @display("i=misc/sensor,p=26.65,119.53");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=178.24,125.53");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=119.08,178.69");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=304.62,107.39");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=373.77,96.00");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=285.55,12.13");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=449.25,33.73");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=217.55,208.23");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=134.08,264.93");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=211.77,290.57");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=325.90,184.91");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=368.76,232.22");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=429.28,165.24");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=356.29,9.58");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=455.01,105.64");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=415.49,280.39");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=456.39,353.26");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=519.06,402.05");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=616.92,410.71");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=660.42,475.07");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=661.41,545.34");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=721.64,505.13");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=803.82,528.73");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=813.73,618.05");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=883.52,637.29");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=904.12,554.69");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=678.49,660.04");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=753.01,655.93");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=750.62,734.75");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=830.12,773.09");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=820.47,847.71");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=877.99,823.17");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=835.62,696.55");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=963.53,591.02");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=963.40,665.01");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=932.40,717.50");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=1007.74,735.92");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=1076.76,770.65");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=1109.54,700.89");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=1054.27,661.29");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=1086.05,582.13");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=1085.01,518.87");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=1115.54,428.49");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=1044.90,438.32");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=1154.51,489.88");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=1024.00,360.27");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=1097.24,306.33");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=1090.45,224.46");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=1008.68,255.16");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=1180.59,312.30");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=1162.28,255.14");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=932.39,316.87");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=905.46,233.74");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=920.86,161.38");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=998.07,142.78");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=1060.34,164.00");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=1077.82,98.55");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=1158.76,119.18");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=1156.51,193.72");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=1009.97,66.08");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=936.11,84.09");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=960.40,374.82");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=825.93,161.25");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=1216.60,184.54");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=1235.29,277.95");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=834.39,392.44");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=761.62,369.38");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=804.65,446.79");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=660.31,766.05");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=602.10,707.12");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=597.49,815.03");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=537.46,796.63");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=592.60,922.63");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=689.04,944.67");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=654.07,877.24");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=618.99,1003.28");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=700.13,1017.13");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=709.77,1108.11");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=631.78,1078.81");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=794.55,1130.04");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=851.71,1104.97");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=922.71,1037.70");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=911.83,1160.79");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=991.39,1170.07");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=1029.69,1221.07");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=1093.77,1279.55");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=1072.51,1349.67");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=1019.39,1290.01");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=1079.48,1186.94");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=935.99,1277.21");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=868.40,1227.04");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=800.67,1224.67");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=720.16,1202.40");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=767.95,1310.16");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=702.57,1337.10");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=818.11,1378.79");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=882.57,1443.45");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=906.61,1367.11");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=950.55,1418.54");
    }

}

