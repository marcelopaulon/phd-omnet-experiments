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

network DADCA_ACK_n100_a8000m2_lpnFn1_6
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
        @display("i=misc/sensor,p=113.94,186.57");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=36.44,196.82");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=160.40,145.40");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=91.94,287.39");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=73.13,375.63");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=204.88,238.29");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=179.22,293.02");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=245.24,344.10");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=240.52,414.31");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=179.20,354.07");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=299.62,169.92");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=376.27,179.05");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=419.89,249.70");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=323.76,242.17");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=343.34,314.46");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=469.00,195.59");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=476.62,287.58");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=459.57,352.22");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=412.72,395.28");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=378.16,473.73");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=352.31,535.10");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=329.80,620.22");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=261.98,625.43");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=260.24,701.52");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=232.63,551.50");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=272.88,496.61");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=305.97,424.90");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=478.69,492.87");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=537.82,450.16");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=575.75,496.83");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=620.85,546.98");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=526.56,537.55");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=407.63,608.01");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=455.62,562.66");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=460.93,721.29");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=380.65,734.13");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=571.47,605.89");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=526.33,665.71");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=630.11,729.87");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=721.51,704.10");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=791.64,662.64");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=853.69,607.79");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=878.44,689.79");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=821.37,734.61");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=748.41,795.70");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=844.89,795.05");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=748.39,867.08");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=798.78,928.20");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=872.95,951.87");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=935.64,911.63");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=865.21,880.92");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=633.53,880.39");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=644.32,806.99");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=581.44,812.09");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=536.23,942.92");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=448.43,988.21");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=520.38,1020.14");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=484.35,1088.66");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=543.10,1144.52");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=412.19,1074.15");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=402.62,1171.44");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=352.14,1205.71");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=601.30,1071.73");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=643.94,953.88");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=693.93,908.03");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=801.13,1046.18");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=760.84,991.80");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=699.00,1046.89");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=981.11,826.49");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=743.54,1087.13");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=828.94,1112.87");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=802.15,1223.38");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=820.52,1296.53");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=686.13,1238.94");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=729.37,1186.11");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=941.84,1001.50");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=996.69,1040.77");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=944.90,1084.42");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=1020.80,1131.97");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=1075.22,1061.80");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=1165.37,1033.91");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=1026.98,983.09");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=1066.11,930.31");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=1041.03,817.54");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=1101.55,841.52");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=1079.10,717.79");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=1127.01,652.08");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=1198.24,654.14");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=1211.18,582.18");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=1236.16,507.88");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=1154.00,476.36");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=1063.23,514.93");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=1036.23,452.97");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=1123.66,556.38");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=953.50,482.05");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=909.36,549.51");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=848.92,522.52");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=816.37,461.88");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=874.75,407.16");
    }

}

