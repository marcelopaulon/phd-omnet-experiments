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

network DADCA_ACK_n100_u4_a8000m2_2
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
        @display("i=misc/sensor;p=498.37,7847.96");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1127.02,7870.50");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2123.67,7802.13");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2887.73,7940.10");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3388.96,7984.72");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5458.70,7775.27");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=829.16,7506.16");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1613.79,7653.69");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2739.62,7556.62");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3178.61,7653.46");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4299.58,7736.06");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=50.10,7419.87");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1303.27,7303.91");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2204.55,7306.99");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3302.25,7265.45");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3926.76,7414.21");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4777.21,7400.90");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5138.42,7353.26");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=690.54,7152.82");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1773.02,7202.52");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3689.48,7108.53");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4084.47,7060.22");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5460.81,7088.07");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5947.01,7074.11");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=267.84,6798.41");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1183.88,6834.09");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1780.01,6771.12");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2278.70,6956.04");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2677.70,6780.92");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3044.00,6834.52");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4850.75,6996.54");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=731.31,6713.24");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3306.67,6543.38");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3607.57,6729.05");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4176.79,6679.97");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4807.31,6551.00");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5319.28,6592.66");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5711.05,6640.51");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1171.75,6445.89");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1716.41,6413.26");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2344.27,6360.00");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2743.72,6377.63");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3880.12,6408.92");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5507.86,6291.45");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=29.32,6053.21");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=427.41,6173.59");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=887.17,6148.49");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1913.27,6065.21");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3615.30,6132.81");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4238.16,6243.33");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5981.94,6105.53");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1495.38,5974.44");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2134.43,5780.38");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3060.23,5997.14");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4562.02,5848.59");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5072.35,5953.85");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=473.90,5658.78");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1182.19,5741.40");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2638.52,5722.20");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3540.03,5585.13");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3948.94,5756.01");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5647.05,5560.15");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2.74,5304.80");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1373.10,5438.59");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1862.38,5459.29");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3023.17,5455.07");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4864.46,5495.27");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5166.99,5312.17");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=951.39,5221.66");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2016.74,5071.25");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2425.59,5189.19");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3084.45,5067.92");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4025.24,5203.41");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4719.04,5042.72");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=464.88,4977.10");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=832.63,4848.92");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1488.27,4901.53");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3682.18,4998.44");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5110.40,4949.02");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2279.86,4536.84");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2896.02,4768.92");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3424.33,4739.63");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4438.96,4767.32");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5460.40,4725.93");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5826.33,4696.40");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=434.28,4506.01");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1655.94,4407.13");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3094.28,4360.41");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3694.81,4481.24");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4046.75,4510.50");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4713.17,4422.47");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5481.97,4372.68");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=221.34,4147.62");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=943.69,4212.95");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1295.21,4236.71");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2015.30,4132.81");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2856.76,4059.82");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4007.16,4145.44");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5182.13,4189.96");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5997.79,4205.82");
    }

}

