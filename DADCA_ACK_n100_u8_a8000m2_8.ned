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

network DADCA_ACK_n100_u8_a8000m2_8
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
        @display("i=misc/sensor;p=518.11,7861.38");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1915.14,7921.49");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2530.09,7791.60");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3597.41,7888.82");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4339.73,7915.55");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4856.31,7850.99");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5195.61,7986.64");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5975.71,7793.25");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1594.28,7696.87");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2043.65,7589.98");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2919.76,7670.71");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3909.91,7524.59");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5537.66,7570.93");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=215.89,7443.30");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=887.03,7283.89");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2452.80,7331.01");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3066.79,7298.28");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3433.24,7398.96");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4533.28,7498.77");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5053.77,7399.18");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1457.02,7065.33");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1829.40,7143.39");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4082.30,7217.50");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5695.88,7182.79");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2247.19,6798.90");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2828.47,6789.14");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3541.41,6949.63");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4501.65,6906.91");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5321.42,6809.44");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5859.56,6822.18");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=103.43,6583.89");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=918.45,6664.45");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1371.56,6693.30");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1776.47,6554.17");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3317.00,6557.99");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4158.48,6642.55");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4848.63,6734.95");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=400.47,6315.52");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2787.04,6302.20");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3933.67,6270.33");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4590.79,6433.31");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=770.00,6081.83");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1273.13,6264.22");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2369.21,6245.78");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3370.22,6175.12");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5113.91,6175.80");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5698.93,6258.09");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=161.85,5897.61");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1504.43,5902.71");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1875.50,5823.31");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3005.79,5992.01");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3532.86,5790.44");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4549.55,6005.24");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5773.78,5832.64");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=754.30,5670.84");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1147.42,5741.01");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2476.92,5718.62");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4005.34,5621.99");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5409.82,5740.60");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=91.45,5466.58");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1520.77,5524.74");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2995.80,5286.44");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4562.38,5464.77");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5037.43,5453.98");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=39.82,5103.04");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=543.74,5165.91");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1039.17,5069.44");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1730.32,5167.27");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2280.08,5252.57");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3539.71,5200.67");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5684.84,5254.31");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1516.05,4868.02");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2058.12,4903.18");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2666.61,5019.78");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3088.12,4890.07");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4063.99,5028.08");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4994.21,4979.48");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=123.31,4727.67");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=623.99,4608.89");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1926.55,4557.28");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3807.49,4729.03");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4338.38,4772.93");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4712.31,4571.36");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5269.66,4569.23");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5735.05,4728.12");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=249.92,4311.65");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1204.87,4402.68");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2326.69,4518.96");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3089.05,4469.64");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4030.12,4404.99");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5880.21,4296.34");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1940.18,4103.11");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2675.29,4160.61");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3648.81,4200.69");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4723.26,4092.21");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5552.99,4139.54");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=735.76,3875.78");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1288.06,3983.18");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2350.82,3892.77");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3616.00,3825.01");
    }

}

