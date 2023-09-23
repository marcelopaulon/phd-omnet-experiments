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

network DADCA_ACK_n100_u2_a8000m2_7
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
        @display("i=misc/sensor;p=81.22,7823.92");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1182.77,7753.97");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1773.19,7978.27");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2525.01,7942.09");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3209.09,7942.56");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3966.99,7980.29");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4580.31,7947.48");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5044.06,7758.05");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5451.79,7967.19");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=728.55,7644.15");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1784.59,7613.69");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2356.65,7553.62");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2959.87,7681.22");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3471.84,7674.52");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3993.04,7567.08");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4384.36,7533.81");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5343.49,7509.36");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5918.23,7573.13");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=464.09,7287.58");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=918.08,7298.84");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1315.61,7350.92");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2713.99,7286.40");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5702.74,7267.04");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=155.12,7082.75");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1996.86,7082.52");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3502.01,7171.70");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4001.39,7030.50");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4786.93,7160.36");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=692.02,6897.68");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1463.87,6935.16");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2250.30,6768.84");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2542.08,6964.60");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2983.19,7000.00");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3265.78,6788.41");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4509.93,6769.77");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=387.80,6599.44");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1531.67,6563.70");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3831.42,6616.96");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4955.29,6618.98");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5659.48,6589.96");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=951.82,6511.20");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1993.64,6361.45");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2820.00,6487.08");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3208.71,6312.51");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3569.06,6384.71");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4499.38,6297.21");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5301.42,6399.53");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1282.84,6054.95");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2380.58,6256.56");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=99.34,5952.70");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=608.01,5909.66");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1921.61,5961.71");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2296.28,5857.40");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2717.88,6000.20");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3766.99,5913.91");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4337.53,5775.97");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4970.54,5866.92");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5657.92,6002.40");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=885.44,5584.15");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3170.38,5741.08");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3763.33,5553.00");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=343.99,5368.69");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2650.11,5450.13");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4588.30,5524.43");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5046.70,5459.14");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5691.66,5523.74");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=711.00,5250.11");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1142.99,5204.06");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2344.77,5214.85");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2989.52,5166.36");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4000.82,5177.69");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4331.74,5061.16");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5859.88,5091.55");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=337.41,4986.24");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=818.63,4867.72");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1702.74,4871.14");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2168.82,4865.90");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2523.54,4828.38");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4809.85,4957.81");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5450.67,4952.07");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=53.11,4540.49");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1290.06,4560.79");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2903.50,4646.95");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3494.89,4754.51");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4012.98,4752.22");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5179.88,4692.12");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5826.19,4739.50");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=518.19,4390.45");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1921.10,4415.96");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4558.44,4519.24");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5223.82,4331.10");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1598.18,4268.01");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2482.29,4183.92");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3458.48,4224.82");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3984.08,4182.56");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4404.93,4153.46");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=344.93,3926.19");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=784.94,3897.42");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1771.33,3947.48");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2880.78,4013.28");
    }

}

