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

network DADCA_ACK_n100_u4_a8000m2_9
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
        @display("i=misc/sensor;p=347.24,7806.12");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=912.06,7980.37");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1517.58,7979.23");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3172.49,7918.56");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4305.37,7946.42");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=815.04,7624.84");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1256.49,7646.58");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2418.53,7644.18");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3894.44,7584.05");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4648.75,7702.82");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5720.26,7617.05");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=352.94,7417.15");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1615.64,7334.96");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2243.04,7332.36");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2917.94,7272.34");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3271.05,7292.31");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4253.82,7394.80");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5111.25,7458.76");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4871.98,7200.23");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5600.39,7034.07");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=217.84,6984.64");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=602.79,6771.63");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1200.27,6875.75");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1629.10,6892.29");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2570.44,6999.96");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3685.60,6824.12");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4183.77,6895.38");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4565.07,6881.54");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5172.75,6827.52");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=108.77,6597.98");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1984.20,6530.54");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2325.05,6638.09");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2815.97,6732.90");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3240.42,6732.92");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=451.85,6395.94");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1541.81,6478.00");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3951.33,6327.96");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4703.66,6485.51");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5329.29,6430.10");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=773.07,6207.45");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1162.31,6182.13");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2345.85,6056.78");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2694.12,6214.86");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3569.24,6159.92");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4251.49,6021.56");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4638.18,6127.71");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5698.57,6153.48");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=168.14,5845.60");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=539.96,5919.68");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1720.17,5898.48");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3230.17,5889.95");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3773.18,5868.96");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5008.65,5994.66");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5332.07,5792.87");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1053.84,5720.61");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1413.17,5647.17");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1952.07,5622.02");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2628.70,5577.21");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2984.96,5555.65");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4117.27,5688.61");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4804.48,5635.37");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=164.08,5487.98");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=718.94,5436.28");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3417.68,5504.56");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5846.29,5296.60");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=493.03,5092.41");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1115.65,5274.06");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1532.00,5132.68");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2304.77,5172.81");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2819.07,5209.31");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3249.36,5076.00");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3685.06,5151.26");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4288.76,5163.22");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5212.47,5196.58");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=141.33,4844.39");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1369.36,4806.91");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1721.70,4785.72");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2670.39,4891.60");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3919.65,4886.15");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4563.14,4922.67");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4914.37,4935.96");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=674.52,4639.64");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1027.60,4713.50");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2105.61,4704.70");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3254.73,4656.56");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5264.21,4780.27");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1726.17,4392.33");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2250.66,4332.33");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2791.05,4376.48");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3664.06,4462.85");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4682.32,4459.67");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5274.61,4309.57");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5899.67,4483.18");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=89.57,4190.29");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=660.02,4287.10");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1102.98,4276.69");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4144.97,4158.52");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5806.39,4111.09");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1700.19,3870.75");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2139.88,3881.44");
    }

}

