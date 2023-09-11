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

network DADCA_ACK_n100_a8000m2_lpnFn1_9
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
        @display("i=misc/sensor,p=66.03,22.86");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=93.37,179.98");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=191.06,198.62");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=227.65,128.93");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=312.76,175.71");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=238.22,235.87");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=149.82,263.79");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=55.95,238.88");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=2.07,136.56");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=65.03,321.69");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=136.04,399.78");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=104.63,468.79");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=200.68,474.76");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=253.18,437.61");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=134.23,578.38");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=89.95,627.77");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=134.68,682.39");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=233.58,697.19");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=304.58,658.65");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=376.17,675.78");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=447.41,710.41");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=361.95,740.40");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=299.65,736.48");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=431.97,830.67");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=498.81,815.75");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=478.54,642.89");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=537.63,619.53");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=596.37,571.34");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=592.68,701.14");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=537.25,746.87");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=674.93,785.06");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=668.06,874.24");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=618.16,839.08");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=589.18,994.77");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=643.55,1062.12");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=698.21,1028.61");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=569.36,1089.32");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=598.34,1161.44");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=660.86,1206.73");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=685.62,1115.39");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=539.52,1195.26");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=491.44,1148.34");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=422.95,1123.83");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=359.75,1127.83");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=293.36,1183.44");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=266.08,1269.64");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=201.47,1288.61");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=130.08,1342.29");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=202.70,1368.98");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=86.98,1264.81");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=148.33,1400.01");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=288.12,1454.36");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=356.12,1443.93");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=452.31,1418.34");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=472.90,1481.72");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=423.60,1529.38");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=338.39,1538.83");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=267.28,1514.84");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=188.54,1507.84");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=373.03,1624.67");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=474.64,1610.97");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=561.05,1626.07");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=563.17,1556.86");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=477.34,1708.21");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=384.63,1704.17");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=428.72,1661.48");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=485.46,1540.49");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=571.22,1458.89");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=512.28,1411.46");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=500.69,1325.34");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=539.08,1264.93");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=438.59,1048.65");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=501.81,1041.40");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=283.61,1091.49");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=195.83,1109.48");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=222.04,1046.53");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=139.61,1070.10");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=72.38,1110.98");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=133.27,1164.61");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=321.52,984.39");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=375.37,939.35");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=376.42,1019.25");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=257.28,992.35");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=236.80,933.75");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=178.48,877.92");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=116.38,942.73");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=259.50,845.89");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=323.23,812.84");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=214.20,770.22");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=462.72,889.82");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=521.88,902.13");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=643.32,941.57");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=710.66,941.21");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=737.86,874.55");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=822.43,899.04");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=791.88,844.08");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=721.44,730.44");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=754.20,794.97");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=871.79,824.98");
    }

}

