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
        @display("i=misc/sensor,p=110.08,196.76");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=141.31,51.16");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=30.61,96.72");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=159.67,147.40");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=233.00,127.28");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=257.45,37.21");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=308.01,75.28");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=360.44,1.35");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=435.99,16.81");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=507.57,50.18");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=421.70,121.71");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=495.70,155.19");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=436.25,196.23");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=317.43,198.71");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=295.09,139.15");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=165.01,273.33");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=182.94,350.89");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=236.27,387.17");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=88.88,294.46");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=120.95,349.31");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=49.20,364.00");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=51.67,431.94");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=27.59,534.06");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=62.97,617.20");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=96.37,530.12");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=162.21,520.70");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=173.91,451.83");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=338.26,437.00");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=402.17,464.12");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=442.13,522.95");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=343.22,525.22");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=366.12,588.28");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=288.76,476.83");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=425.21,352.25");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=507.91,366.11");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=366.29,338.87");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=489.13,434.91");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=520.49,597.43");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=520.28,518.06");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=430.73,664.32");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=600.10,675.66");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=535.37,724.70");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=474.84,746.86");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=387.39,716.43");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=356.30,768.64");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=454.04,814.01");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=581.00,503.01");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=631.10,575.22");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=734.09,547.63");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=755.49,612.17");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=807.30,451.85");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=761.33,493.03");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=820.90,532.26");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=888.55,465.41");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=960.64,564.16");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=1004.98,474.77");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=790.39,354.96");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=878.53,358.75");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=1064.78,425.98");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=1132.66,405.61");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=1193.90,329.77");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=885.63,526.95");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=1061.44,582.37");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=1023.78,629.58");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=1152.75,587.20");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=1215.56,641.70");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=1297.96,695.10");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=1391.79,677.27");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=1474.52,706.30");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=1490.25,794.36");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=1415.74,775.03");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=1417.81,575.98");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=1466.19,528.03");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=1423.29,459.47");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=1349.68,504.27");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=1252.79,521.57");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=1295.65,582.74");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=992.70,689.29");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=954.70,624.83");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=956.02,423.05");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=960.96,362.09");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=796.51,239.87");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=850.86,265.88");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=704.12,364.89");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=677.94,438.97");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=657.39,257.95");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=602.75,202.90");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=556.29,256.30");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=677.14,521.02");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=879.18,612.43");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=859.03,695.16");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=817.01,618.66");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=1068.42,704.69");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=1009.38,781.42");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=1075.98,830.40");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=1024.52,891.96");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1009.44,982.53");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=955.12,947.45");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=948.89,780.02");
    }

}

