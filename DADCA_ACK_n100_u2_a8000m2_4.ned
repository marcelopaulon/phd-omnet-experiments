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

network DADCA_ACK_n100_u2_a8000m2_4
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
        @display("i=misc/sensor;p=571.00,7784.08");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1093.12,7763.05");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2662.20,7919.23");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3452.58,7756.60");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3960.03,7889.57");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4483.52,7862.70");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5265.69,7960.57");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1513.04,7740.76");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2022.78,7605.53");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1507.09,7272.84");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2570.37,7267.49");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3031.69,7496.04");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3686.52,7428.38");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4354.89,7348.92");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4900.10,7494.25");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5472.28,7380.87");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=6.48,7256.95");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=449.20,7157.51");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1108.65,7181.87");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1912.15,7057.87");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3049.71,7131.04");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3405.92,7098.87");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2498.67,6790.15");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3925.29,6871.15");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4293.40,6909.65");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4871.84,6956.93");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5304.28,6774.46");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=254.34,6724.35");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=919.72,6604.46");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1403.59,6675.78");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2142.11,6679.80");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2984.20,6630.96");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3357.48,6637.59");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=395.90,6304.31");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1444.67,6291.57");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2681.03,6359.74");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4201.77,6473.15");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4561.93,6379.75");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5033.39,6453.35");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5901.85,6466.33");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=6.55,6196.74");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1016.54,6256.30");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1735.76,6060.13");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2184.53,6231.35");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3069.38,6244.51");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3814.78,6212.51");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4176.23,6023.37");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5279.83,6108.35");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1233.18,5888.05");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2519.65,5918.27");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3240.01,5875.77");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4902.16,5959.15");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5697.98,5992.48");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=431.16,5692.38");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=804.46,5607.97");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2305.44,5608.93");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2856.34,5607.39");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3841.44,5712.49");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4439.96,5728.98");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5316.02,5639.03");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1696.59,5521.74");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3603.67,5416.66");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4183.25,5351.14");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4928.77,5438.49");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5904.52,5334.17");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=122.25,5276.33");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1447.86,5229.37");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1963.41,5160.16");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2488.02,5219.03");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2976.80,5255.42");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4567.72,5240.83");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5503.74,5078.59");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=120.05,4902.24");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=481.85,4911.79");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1071.45,5006.30");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1692.86,4791.14");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3148.12,4784.44");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3675.76,5001.83");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4907.65,4794.17");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1366.13,4631.80");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2072.45,4730.61");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2593.72,4547.67");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3676.84,4592.29");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4130.50,4760.38");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4492.40,4705.10");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5832.58,4606.76");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=70.40,4462.95");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=595.44,4521.73");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=974.65,4460.67");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2171.29,4317.91");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3328.68,4402.29");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1168.99,4125.97");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1781.53,4130.83");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2540.05,4128.65");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2996.42,4110.30");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3994.66,4285.73");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4476.56,4060.17");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5092.32,4230.55");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5588.97,4117.72");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=719.10,3952.92");
    }

}

