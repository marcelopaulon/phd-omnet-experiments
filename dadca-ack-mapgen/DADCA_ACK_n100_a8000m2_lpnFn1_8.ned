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
        @display("i=misc/sensor,p=26.66,117.03");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=7.30,39.80");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=71.74,235.19");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=147.71,232.72");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=101.40,180.78");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=2.38,191.20");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=194.98,82.80");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=282.92,124.10");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=319.88,69.68");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=280.05,22.83");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=191.29,19.09");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=223.03,172.70");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=64.31,14.37");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=125.48,32.18");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=373.98,125.21");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=424.95,46.01");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=439.77,210.70");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=462.49,291.28");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=366.63,251.17");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=368.86,321.68");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=406.50,400.03");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=550.88,288.86");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=463.93,367.01");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=486.01,429.75");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=533.78,491.08");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=468.47,515.79");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=387.07,483.22");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=346.57,407.42");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=275.75,409.68");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=254.00,337.98");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=568.03,430.81");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=478.80,606.67");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=387.35,553.12");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=620.71,512.25");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=547.27,363.30");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=501.39,116.98");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=561.73,130.66");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=602.33,75.38");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=682.03,78.06");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=641.94,125.49");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=244.02,239.18");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=211.33,384.46");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=215.81,445.69");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=165.86,507.93");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=257.63,498.12");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=619.64,291.78");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=638.56,221.82");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=708.01,229.16");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=711.88,301.06");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=771.01,264.05");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=766.06,333.70");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=795.70,428.45");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=853.45,493.39");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=940.24,492.62");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=1022.53,464.07");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=974.99,425.72");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=551.88,11.78");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=631.24,21.53");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=707.63,140.69");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=676.28,355.85");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=674.43,445.54");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=743.01,466.53");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=900.77,405.84");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=989.96,566.64");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=920.91,552.89");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=1054.16,400.40");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=1094.03,458.73");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=1174.91,436.13");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=1175.21,522.64");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=1224.92,475.16");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=1269.89,516.05");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=1207.64,577.81");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=1285.83,590.65");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=1326.05,673.83");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=1389.84,642.09");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=1337.65,526.92");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=1190.04,677.80");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=1207.54,739.62");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=1120.29,766.61");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=1174.64,795.85");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=1180.84,882.44");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=1112.06,901.32");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=1030.36,926.26");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=970.68,911.51");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=923.89,789.03");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=1004.82,742.55");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=1025.99,805.48");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=1099.86,704.10");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=1069.18,622.92");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=1012.19,667.28");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=1269.95,839.77");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=1337.36,789.82");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=1392.64,740.11");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=1488.23,647.72");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=1457.93,581.47");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=1447.17,504.95");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1446.23,771.15");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=1114.17,375.12");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=1192.22,320.06");
    }

}

