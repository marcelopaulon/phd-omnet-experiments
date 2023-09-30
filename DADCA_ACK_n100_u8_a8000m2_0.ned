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

network DADCA_ACK_n100_u8_a8000m2_0
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
        @display("i=misc/sensor;p=318.25,7957.95");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=785.42,7857.62");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1408.14,7806.13");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2683.22,7868.67");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3489.03,7879.66");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3842.47,7809.69");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4531.64,7788.92");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5006.18,7779.39");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5538.04,7983.71");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1952.72,7748.54");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2435.42,7548.53");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3046.72,7542.24");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5923.45,7505.09");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=57.18,7399.64");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1673.52,7403.74");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3414.62,7430.97");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4206.06,7374.92");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5425.51,7359.46");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=451.52,7025.42");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1130.68,7256.67");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2295.30,7218.58");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2953.82,7173.52");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3641.86,7025.78");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4811.79,7241.90");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=77.93,6847.15");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=982.80,6797.94");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1326.00,6966.00");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2043.78,6809.63");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5037.15,6938.97");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5658.42,6902.21");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=594.81,6592.69");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2595.00,6721.53");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3159.57,6677.52");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3658.71,6525.96");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4057.67,6523.61");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4354.12,6732.69");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=32.82,6472.40");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1057.90,6411.38");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1564.56,6302.75");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2029.83,6405.39");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4346.28,6272.55");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4891.59,6462.16");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5332.75,6509.47");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=673.22,6234.85");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1310.31,6045.50");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2383.78,6140.51");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3077.39,6164.14");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3603.01,6144.60");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4045.40,6049.25");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4919.17,6060.93");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5793.40,6059.10");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=310.19,5956.30");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1924.08,5972.49");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5319.49,5916.14");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=765.56,5705.14");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1496.08,5710.39");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2301.03,5594.52");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2681.71,5627.51");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3165.20,5602.83");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3954.00,5596.93");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4557.05,5619.43");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5126.71,5605.65");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5657.60,5655.00");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=993.00,5391.82");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1348.94,5367.44");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1852.34,5502.23");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3582.37,5475.74");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5452.67,5361.83");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=41.84,5219.49");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=426.20,5268.47");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1047.12,5031.18");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2285.48,5163.77");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2859.51,5183.69");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3548.87,5076.07");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4065.16,5121.72");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4829.81,5160.59");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=218.53,4819.71");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=575.24,4912.87");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1474.25,4927.16");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5319.60,4947.67");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1099.14,4603.77");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1832.86,4721.06");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2510.43,4732.47");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3098.63,4655.49");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3431.57,4536.03");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3894.76,4654.96");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4833.59,4650.40");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5258.46,4581.55");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5740.25,4614.26");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=126.59,4347.00");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=562.78,4504.26");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1369.35,4370.16");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1750.68,4362.27");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4455.99,4508.80");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=807.58,4119.97");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2613.29,4220.50");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3163.04,4186.40");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4083.07,4226.81");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4665.44,4168.94");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5049.37,4230.71");
    }

}

