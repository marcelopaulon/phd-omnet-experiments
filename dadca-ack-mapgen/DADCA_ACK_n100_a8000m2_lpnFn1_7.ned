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

network DADCA_ACK_n100_a8000m2_lpnFn1_7
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
        @display("i=misc/sensor,p=96.09,164.59");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=35.77,83.93");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=175.12,293.19");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=180.81,382.32");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=276.32,371.59");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=294.57,429.39");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=76.98,421.73");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=138.84,474.35");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=140.09,571.60");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=183.48,617.76");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=91.38,638.62");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=9.92,651.44");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=16.40,539.93");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=137.02,749.73");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=92.64,798.09");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=23.58,770.56");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=171.10,814.95");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=233.27,803.05");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=63.35,725.01");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=263.94,634.48");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=154.37,677.61");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=241.93,698.21");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=307.81,559.99");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=356.69,417.09");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=197.26,440.54");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=419.52,381.22");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=418.19,296.44");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=66.12,332.33");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=21.90,218.76");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=119.05,224.44");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=2.63,137.14");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=184.60,161.80");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=196.38,224.07");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=241.58,269.87");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=248.46,117.93");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=175.89,83.15");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=3.57,300.63");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=15.53,448.82");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=362.83,487.44");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=482.66,392.83");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=224.05,547.36");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=335.77,656.34");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=401.78,612.95");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=109.82,17.67");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=249.62,476.37");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=314.12,845.31");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=315.00,714.56");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=443.40,672.11");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=488.16,733.73");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=500.23,828.12");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=515.55,922.06");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=535.07,1009.19");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=603.79,1024.87");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=641.83,949.19");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=619.01,891.74");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=669.68,819.81");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=739.15,853.28");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=825.18,902.35");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=854.83,993.26");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=903.47,1070.03");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=986.73,1050.93");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=974.98,1158.59");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=1046.40,918.20");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=1125.52,967.23");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=1051.24,1031.36");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=1206.39,989.20");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=1280.11,1023.72");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=1249.34,937.34");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=1262.77,849.97");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=1212.41,790.84");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=1189.65,722.57");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=1145.52,589.09");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=1057.58,570.58");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=976.49,552.55");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=922.64,603.29");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=850.99,498.06");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=915.31,540.58");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=983.20,704.02");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=1037.96,653.97");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=1203.52,633.86");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=1101.57,677.40");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=1187.88,531.81");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=1132.71,506.32");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=1272.84,662.18");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=1335.16,802.00");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=1467.04,805.25");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=1468.58,718.95");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=1395.10,767.12");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=1414.57,849.77");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=1377.98,669.99");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=1429.11,589.05");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=1520.25,622.73");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=1434.17,915.29");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=1459.19,982.37");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=1430.50,1068.75");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=1348.62,1119.18");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1364.42,1058.88");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=1346.60,970.43");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=1525.69,950.67");
    }

}

