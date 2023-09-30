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

network DADCA_ACK_n100_u16_a8000m2_5
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
        @display("i=misc/sensor;p=268.55,7945.68");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=859.46,7792.40");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1546.96,7840.36");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4111.11,7761.85");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5124.31,7996.58");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=79.62,7509.98");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1112.16,7528.08");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2440.94,7582.51");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2951.56,7510.72");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3432.40,7563.47");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4829.74,7526.19");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5856.06,7664.38");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=423.68,7387.84");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3813.05,7424.08");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4165.65,7326.01");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5314.23,7346.79");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1480.31,7147.40");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2149.11,7122.61");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2832.44,7074.37");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3382.52,7157.90");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4554.17,7095.41");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5685.03,7118.78");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=354.77,6877.49");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=844.46,6932.29");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1226.03,6776.22");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1830.80,6926.52");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2481.01,6849.94");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3531.34,6789.57");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4221.46,6902.75");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5876.00,6793.71");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3056.65,6622.99");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4006.62,6549.68");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4673.01,6552.34");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4985.60,6751.89");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5467.96,6709.21");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=807.61,6442.85");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2569.41,6497.28");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3475.79,6442.47");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4425.25,6287.31");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5324.48,6340.10");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=254.93,6185.03");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1291.92,6188.29");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1961.50,6247.39");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2428.63,6106.82");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3279.26,6066.90");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3687.06,6072.12");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4039.19,6058.87");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4940.94,6166.05");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=894.70,5953.70");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2961.22,5895.42");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4598.79,5841.03");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5612.79,6007.32");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=338.81,5753.52");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1164.84,5589.87");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1697.33,5709.95");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2429.16,5737.74");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2878.16,5536.45");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3553.82,5585.78");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4187.00,5664.23");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5129.09,5573.28");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5514.95,5647.26");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=784.24,5383.52");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2245.33,5344.58");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4667.53,5435.86");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1419.27,5056.83");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2598.59,5232.82");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3410.93,5207.65");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3977.40,5182.83");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5819.17,5100.09");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=118.38,4834.26");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=479.47,4994.46");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1020.84,4837.85");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1944.40,4960.79");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2418.25,4799.38");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3060.75,5007.95");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3642.87,4913.58");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4402.35,4927.59");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5067.07,4927.43");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2856.15,4706.13");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3329.06,4698.35");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3835.22,4583.54");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4904.71,4545.63");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5559.57,4573.92");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=113.35,4450.85");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=521.35,4359.21");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=862.78,4460.01");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1553.79,4381.90");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4048.26,4298.00");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4500.12,4428.26");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5911.95,4483.92");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2185.40,4133.20");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2748.85,4251.27");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3303.73,4230.86");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5269.03,4109.42");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=74.29,3825.01");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=420.53,3918.50");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=838.52,3839.87");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1788.92,3950.18");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2425.10,3867.82");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3515.11,3922.10");
    }

}

