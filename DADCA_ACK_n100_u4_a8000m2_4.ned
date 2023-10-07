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

network DADCA_ACK_n100_u4_a8000m2_4
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
        @display("i=misc/sensor;p=404.57,7963.47");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=914.86,7971.21");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1539.40,7842.48");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4265.40,7839.54");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=276.66,7633.72");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=838.47,7622.81");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2063.12,7611.31");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2498.55,7610.15");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3049.49,7653.48");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3693.76,7741.84");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4708.60,7644.93");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5061.95,7638.36");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5646.50,7643.81");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=208.30,7267.85");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2794.22,7410.01");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3235.11,7307.88");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3814.42,7276.63");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4199.82,7483.17");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=841.86,7183.15");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1805.66,7081.37");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2374.21,7120.81");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2966.53,7055.95");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4496.14,7202.49");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5112.66,7200.71");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5753.81,7241.58");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=508.24,6902.48");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1471.06,6902.40");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2694.21,6812.19");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3837.33,6898.01");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4277.22,6786.12");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4688.48,6863.46");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5543.25,6883.97");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=93.29,6761.59");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=731.79,6573.65");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1750.78,6656.61");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2221.88,6727.76");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=431.80,6333.50");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1272.71,6299.84");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2020.55,6417.31");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2529.85,6374.06");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3108.69,6388.36");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3510.58,6359.55");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3899.94,6450.92");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5007.35,6450.27");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5357.91,6370.89");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5941.11,6287.26");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1714.36,6151.26");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4459.01,6184.59");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4826.09,6085.42");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=460.67,5961.80");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1090.35,5826.99");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2091.68,5773.94");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3438.47,6002.31");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3795.17,5803.36");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5374.02,5859.63");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=797.01,5547.59");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1602.95,5634.82");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3098.04,5623.82");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3508.77,5558.03");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4226.34,5739.04");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4813.56,5724.54");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5924.74,5622.39");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=374.76,5509.46");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1142.71,5429.53");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2009.32,5400.49");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2589.40,5408.70");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4518.69,5520.11");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5431.98,5487.34");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1054.68,5032.32");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2513.70,5062.12");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3605.79,5172.61");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4014.99,5251.48");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4390.38,5145.14");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4734.24,5243.00");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5134.93,5181.93");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5582.40,5041.59");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=567.23,4912.63");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1651.64,5009.74");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2910.42,4915.63");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3404.53,4838.20");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3982.35,4896.38");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4668.24,4896.32");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5055.18,4799.56");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1414.85,4617.27");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2581.02,4567.45");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4316.46,4777.42");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5449.80,4677.17");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5966.39,4577.67");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=528.71,4506.30");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=934.34,4475.69");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2070.16,4500.14");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2876.69,4359.96");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3410.95,4451.44");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=136.24,4180.78");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1084.97,4100.05");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2593.25,4050.59");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3860.52,4239.75");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4474.62,4175.41");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5060.64,4108.83");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5872.92,4133.41");
    }

}

