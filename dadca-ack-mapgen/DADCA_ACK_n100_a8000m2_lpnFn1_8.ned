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
        @display("i=misc/sensor,p=48.02,159.63");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=90.55,221.02");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=176.07,233.52");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=210.12,307.95");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=140.63,327.22");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=236.90,249.94");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=157.62,167.99");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=188.64,109.05");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=237.21,178.46");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=322.79,140.65");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=282.19,353.64");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=239.84,407.02");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=175.71,437.16");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=254.37,486.80");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=185.10,513.75");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=117.02,584.12");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=26.45,588.08");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=76.39,668.43");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=78.72,747.87");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=13.32,753.19");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=58.47,847.78");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=122.42,794.18");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=184.58,801.35");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=149.04,850.94");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=237.79,934.01");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=191.76,979.28");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=276.65,726.62");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=330.89,700.83");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=421.06,690.03");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=467.57,732.10");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=499.86,801.48");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=436.43,792.65");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=570.39,685.40");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=626.81,649.48");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=645.70,568.06");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=717.93,571.20");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=756.11,647.66");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=800.18,606.57");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=822.57,688.19");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=895.08,649.29");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=986.40,636.21");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=1054.17,705.59");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=1085.51,638.69");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=1099.30,554.90");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=1161.08,597.96");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=1199.71,678.38");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=1162.11,729.74");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=1251.59,719.34");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=1262.79,658.82");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=959.20,534.43");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=866.57,513.92");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=931.33,447.89");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=850.21,412.67");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=746.54,439.04");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=841.90,294.43");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=878.85,350.57");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=767.66,377.12");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=865.17,175.32");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=892.20,254.77");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=975.59,272.71");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=1066.80,163.14");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=982.79,210.13");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=1066.78,226.43");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=1197.43,105.35");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=1128.98,157.69");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=1154.99,230.56");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=1223.51,258.87");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=1242.00,185.12");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=1196.32,358.13");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=1172.74,413.56");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=1233.91,430.20");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=1155.91,476.03");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=1369.85,497.47");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=1424.98,424.58");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=1345.93,404.49");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=1439.59,338.47");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=1492.01,368.68");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=1483.88,478.34");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=1561.24,489.06");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=1536.46,227.55");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=1588.90,304.52");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=1663.32,353.90");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=1691.22,294.40");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=1761.77,244.04");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=1615.36,190.15");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=1549.43,140.26");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=1572.40,55.14");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=1629.13,17.44");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=1519.32,22.64");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=1619.08,101.53");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=1677.02,71.82");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=1699.13,183.60");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=1779.97,149.86");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=1804.22,324.01");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=1915.49,262.22");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=1907.16,195.36");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1746.61,85.83");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=1579.68,424.01");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=1484.65,196.15");
    }

}

