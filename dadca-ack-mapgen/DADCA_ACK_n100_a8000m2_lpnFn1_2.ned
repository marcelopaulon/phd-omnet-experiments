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

network DADCA_ACK_n100_a8000m2_lpnFn1_2
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
        @display("i=misc/sensor;p=67.17,33.69");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3.43,37.95");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=153.89,31.12");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=111.72,207.31");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=52.60,243.69");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=37.71,136.18");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=185.35,165.81");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=265.81,198.51");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=333.90,192.02");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=367.39,248.26");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=447.63,215.74");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=458.81,131.23");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=456.18,58.73");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=486.82,263.77");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=570.95,292.41");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=652.52,304.15");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=732.05,276.16");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=792.60,319.80");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=854.97,256.34");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=953.14,230.05");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=954.30,303.04");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1017.34,328.26");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1051.45,391.45");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=964.64,367.81");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=786.90,446.62");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=857.43,409.02");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=925.83,505.06");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=985.61,438.32");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1081.11,445.44");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1129.75,531.45");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1112.81,602.44");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1222.22,584.57");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1238.09,489.38");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1155.18,461.73");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1192.55,376.26");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1192.58,298.71");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1136.29,259.50");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1275.44,308.70");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1125.21,327.76");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=819.91,155.05");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=750.18,146.94");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=751.12,68.35");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=679.71,172.25");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=626.68,108.20");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=545.42,148.82");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=747.15,359.11");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=834.27,512.72");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=922.00,417.21");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1077.32,175.27");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1137.01,102.89");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1098.25,53.08");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1226.35,176.29");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1239.76,108.01");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1283.92,30.25");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1344.17,60.37");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1419.46,46.34");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1448.40,174.60");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1499.04,104.00");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1470.97,235.53");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1387.30,226.58");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1307.36,246.79");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1321.56,163.94");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1183.34,42.71");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=994.65,83.86");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=933.86,81.08");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=883.37,45.18");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=823.20,29.47");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=689.46,107.28");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=613.75,168.12");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=808.93,91.39");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=986.64,5.47");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1017.59,188.16");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1045.50,123.36");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1063.65,265.75");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1041.61,32.74");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=730.94,555.81");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=858.71,620.35");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=791.55,642.70");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=740.58,676.80");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=822.92,697.64");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=771.08,745.25");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=804.39,817.45");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=840.42,882.77");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=879.88,932.65");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=725.91,858.07");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=657.94,791.14");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=646.64,660.92");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=625.29,592.38");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=551.42,839.85");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=475.98,852.61");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=364.52,796.84");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=364.51,726.70");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=366.01,648.83");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=403.01,884.31");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=433.18,1003.47");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=402.71,944.88");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=334.64,910.45");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=317.15,971.41");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=250.45,820.54");
    }

}

