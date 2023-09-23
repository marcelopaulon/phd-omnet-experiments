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

network DADCA_ACK_n100_u16_a8000m2_7
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
        @display("i=misc/sensor;p=1071.31,7788.47");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1501.73,7913.36");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2124.99,7985.29");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3507.20,7799.63");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3927.95,7952.34");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5284.37,7999.69");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1433.06,7513.50");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1800.68,7625.02");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2390.26,7632.12");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3063.38,7735.87");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3822.33,7591.23");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4516.18,7648.77");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5189.95,7647.92");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=488.05,7382.66");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1138.77,7284.87");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1998.88,7265.63");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2918.57,7273.23");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4036.96,7312.72");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5036.59,7285.58");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5758.13,7346.05");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=342.64,7013.45");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1606.52,7196.82");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3494.51,7074.03");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4480.85,7083.14");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5434.00,7107.67");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1357.37,6766.19");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2554.49,6978.14");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3140.75,6793.87");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4141.44,6903.41");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4734.40,6787.31");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5089.04,6824.93");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=382.32,6518.95");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=755.17,6626.44");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1898.21,6686.54");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3495.65,6667.36");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3896.16,6610.03");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5481.22,6728.30");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1307.81,6397.50");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2413.08,6461.14");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3095.73,6342.74");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4228.22,6489.88");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4633.75,6364.50");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=730.29,6242.72");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1707.68,6145.41");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2195.48,6186.52");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3423.64,6166.53");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5152.98,6257.99");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5502.40,6048.56");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=42.42,5974.14");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1327.96,5945.16");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2955.95,6012.19");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3900.65,5925.38");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4289.35,6005.38");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4702.40,5958.13");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=132.48,5533.03");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=750.79,5667.66");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1131.85,5541.49");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1544.15,5649.33");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1953.01,5643.23");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2412.80,5713.67");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3084.30,5659.05");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3482.04,5552.59");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5195.85,5655.09");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5932.62,5539.94");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2760.20,5373.62");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3860.81,5427.60");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4490.75,5477.33");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5370.11,5313.96");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=359.35,5221.67");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=797.76,5122.57");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1458.89,5247.91");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1984.81,5238.50");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2368.53,5058.16");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5010.62,5097.82");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=257.70,4789.92");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1576.94,4864.53");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2695.20,4881.11");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3095.64,4799.93");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3478.63,4911.17");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4255.03,4916.15");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5271.63,4825.56");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5666.47,4924.09");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1121.05,4645.81");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1975.09,4731.40");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2450.75,4540.73");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3908.32,4601.10");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4650.43,4781.56");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5948.90,4600.85");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=128.22,4426.08");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1575.49,4476.84");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2953.63,4412.12");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3319.61,4475.08");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4338.92,4490.01");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5282.75,4415.11");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=902.33,4071.03");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2007.02,4105.26");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3752.49,4120.23");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4687.78,4190.41");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5781.77,4243.76");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=191.11,3864.23");
    }

}

