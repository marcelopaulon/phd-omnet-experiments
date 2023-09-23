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

network DADCA_ACK_n100_u2_a8000m2_0
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
        @display("i=misc/sensor;p=212.31,7922.13");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=991.61,7851.61");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1393.85,7981.73");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3049.37,7828.25");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5857.30,7777.85");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=701.04,7645.30");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1299.42,7593.82");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1935.07,7532.41");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2568.20,7654.15");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3444.43,7583.38");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3898.55,7721.19");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5114.45,7751.78");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=488.77,7361.90");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=952.74,7257.68");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1560.58,7288.77");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2267.47,7389.74");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3062.74,7280.01");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4778.79,7432.16");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5407.14,7457.48");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=231.96,7089.79");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3413.57,7034.40");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3756.56,7238.35");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5932.40,7192.95");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=711.93,6853.83");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1329.98,6925.83");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1801.53,6878.37");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2149.28,6957.19");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2609.12,7005.13");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4825.99,6957.41");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5476.65,6937.43");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5838.43,6815.53");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1049.40,6691.08");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2639.12,6527.82");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3001.88,6678.16");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3773.02,6683.08");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4156.06,6741.14");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=168.32,6454.08");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=711.53,6298.03");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1346.94,6491.02");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1665.71,6289.41");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3134.35,6278.00");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4399.20,6468.12");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4771.81,6302.99");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5335.20,6438.82");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2327.91,6204.59");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4030.67,6236.41");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=49.60,5919.90");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=859.14,5867.96");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1293.98,5821.37");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1723.86,5795.67");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3076.73,5867.27");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3505.59,5797.04");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4112.11,5878.74");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4667.22,5956.29");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5014.67,5866.24");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2516.54,5567.38");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3957.23,5533.57");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4445.31,5538.77");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=386.51,5497.11");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=763.83,5482.27");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1281.18,5402.16");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1730.92,5356.42");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2123.43,5481.90");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3046.86,5436.35");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3541.65,5328.67");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5304.53,5367.75");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5748.43,5472.38");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=110.51,5262.50");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=966.83,5065.29");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2291.40,5154.59");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2814.62,5064.44");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4029.45,5101.78");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5052.28,5091.65");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=540.11,4875.17");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1320.37,4870.18");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1835.14,4996.95");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2174.94,4799.23");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3249.33,4969.54");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3617.06,4972.35");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4693.57,4982.65");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5283.66,4812.23");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5565.68,5024.10");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3565.93,4618.39");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4296.09,4550.74");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4870.57,4608.22");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5780.78,4651.72");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=147.82,4429.32");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=498.25,4458.45");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1621.14,4521.70");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2075.48,4436.58");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2706.34,4427.77");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3906.08,4491.46");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=798.87,4251.45");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1769.42,4069.88");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3152.94,4195.25");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3684.55,4047.35");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4080.35,4163.49");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4529.87,4105.51");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5196.07,4134.20");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5894.86,4204.39");
    }

}

