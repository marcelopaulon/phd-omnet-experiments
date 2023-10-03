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
        @display("i=misc/sensor;p=1481.07,7757.40");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1792.83,7987.34");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2982.47,7864.59");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4179.93,7968.64");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4852.30,7861.02");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5733.73,7799.78");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=441.90,7517.18");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1092.77,7642.26");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1772.49,7562.32");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2229.38,7585.49");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3650.25,7725.32");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4301.55,7547.36");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5291.19,7690.42");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2481.86,7262.41");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3371.81,7339.45");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3938.52,7451.98");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4795.63,7495.17");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5391.41,7290.65");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5843.24,7416.68");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1230.64,7199.83");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1619.93,7147.10");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2847.67,7240.92");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4871.48,7097.16");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=25.07,6945.14");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=518.48,6991.79");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=999.88,6925.14");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2134.13,6963.47");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3435.73,6975.60");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3754.47,6803.66");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4513.30,6825.56");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5480.80,6861.03");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1266.99,6668.56");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1774.57,6708.78");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2511.32,6718.40");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2990.89,6515.69");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4293.43,6538.72");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4926.04,6594.41");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5926.26,6594.82");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=494.32,6499.69");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1543.41,6284.19");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2178.12,6376.76");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3398.52,6386.41");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5477.06,6465.57");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=833.15,6099.64");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2364.87,6051.58");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3037.36,6154.38");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4490.90,6240.52");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5014.82,6189.55");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5691.74,6063.76");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=474.99,5804.85");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1273.39,5855.49");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1709.82,5910.16");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3623.24,5950.21");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5416.09,5781.64");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2149.59,5761.82");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2676.04,5685.86");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3148.52,5772.38");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4478.10,5578.39");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4843.44,5647.94");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=9.58,5368.78");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=616.95,5354.43");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1046.40,5483.16");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1704.48,5514.45");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3372.19,5491.76");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3746.88,5390.80");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5174.18,5395.22");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5799.91,5283.73");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=963.25,5104.16");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1534.14,5104.32");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2129.71,5143.75");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2766.79,5189.45");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3190.97,5174.34");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4205.76,5113.48");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4684.68,5164.11");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=113.07,4887.61");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2460.11,4848.49");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3086.82,4787.16");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3517.55,5024.17");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3859.56,4927.20");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5511.11,4999.28");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=546.12,4729.51");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4646.76,4639.25");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5089.52,4578.92");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5525.65,4561.32");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=163.17,4294.55");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=874.16,4443.85");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1293.00,4527.09");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1753.24,4525.48");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2293.29,4491.93");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2924.14,4432.21");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3315.72,4506.05");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3858.98,4462.49");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1014.01,4094.60");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1520.47,4157.44");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2581.26,4171.58");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3574.39,4108.81");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4354.45,4249.06");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4875.55,4267.57");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5441.70,4090.01");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5834.52,4085.26");
    }

}

