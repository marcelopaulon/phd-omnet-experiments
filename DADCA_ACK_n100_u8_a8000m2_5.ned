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

network DADCA_ACK_n100_u8_a8000m2_5
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
        @display("i=misc/sensor;p=2032.07,7818.54");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=4331.18,7820.70");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=5041.00,7975.10");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5725.70,7958.97");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=533.12,7543.20");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1018.08,7725.81");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2625.72,7572.17");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3276.38,7671.23");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3747.05,7684.41");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5387.26,7510.89");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5802.79,7549.36");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1674.84,7494.47");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4605.76,7433.59");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5008.94,7489.43");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=310.37,7190.66");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1363.52,7256.32");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1940.73,7091.85");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2289.53,7188.51");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2665.61,7199.56");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3275.55,7126.56");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3956.50,7240.73");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4896.55,7118.78");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5324.83,7062.78");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5968.79,7089.32");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=985.36,6939.66");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3874.62,6767.52");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4533.83,6964.67");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=274.97,6746.65");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1810.91,6613.65");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2278.93,6698.70");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2954.10,6709.26");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4469.02,6619.30");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4956.89,6696.63");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5437.41,6647.50");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=814.80,6345.85");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1195.64,6363.55");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4757.38,6389.73");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5473.33,6294.65");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5962.85,6297.93");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=276.28,6228.42");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1453.56,6121.31");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1886.93,6262.05");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2316.07,6173.53");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3210.12,6209.26");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3578.68,6246.64");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4077.49,6235.51");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5153.68,6122.93");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=7.82,5817.63");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=648.41,5838.12");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2186.27,5796.52");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2874.92,5903.26");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3764.06,5810.53");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4122.93,5782.66");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4726.83,5914.16");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5742.39,5909.37");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1144.31,5745.12");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3096.34,5603.23");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5199.67,5722.34");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=186.07,5278.70");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=870.61,5409.73");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1345.10,5418.77");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1720.39,5480.81");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2651.37,5446.61");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3436.40,5373.95");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4071.13,5334.69");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4565.75,5482.77");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4929.38,5352.13");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5754.64,5377.49");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1751.44,5119.43");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2180.91,5178.20");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3064.89,5231.33");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3710.57,5153.64");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5126.37,5035.66");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=650.75,4827.21");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=975.71,4993.20");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1361.06,4811.00");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3350.19,4867.11");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4454.62,5019.49");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5502.42,4905.80");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5923.70,4912.06");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=319.05,4689.20");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2056.21,4737.18");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2620.46,4778.39");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2971.32,4594.51");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3922.26,4713.74");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4448.42,4598.49");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=31.67,4462.21");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1022.49,4531.66");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1659.53,4433.25");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2300.44,4486.20");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2769.63,4304.09");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3255.16,4327.96");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5617.54,4389.32");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5996.28,4461.34");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=469.05,4096.41");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4374.65,4185.56");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4980.55,4198.36");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=808.73,3980.64");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1448.13,3992.28");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2049.64,3952.05");
    }

}

