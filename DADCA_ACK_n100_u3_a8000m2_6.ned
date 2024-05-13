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

network DADCA_ACK_n100_u3_a8000m2_6
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
        @display("i=misc/sensor;p=203.47,7896.02");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=531.42,7753.73");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1411.95,7795.15");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2645.36,7923.90");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3777.65,7756.11");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4389.56,7959.29");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4848.40,7849.12");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5171.89,7990.30");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1784.25,7687.91");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2159.17,7712.32");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3292.06,7585.12");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5758.52,7699.47");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=235.11,7454.45");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=559.45,7277.23");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1042.77,7445.87");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2842.76,7413.15");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3706.87,7269.98");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4315.58,7467.09");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4732.84,7298.63");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5326.08,7483.26");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=33.57,7067.96");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1596.81,7202.69");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2080.83,7222.83");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2401.29,7011.62");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3086.84,7142.86");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4244.28,7111.31");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5124.19,7072.28");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5503.14,7110.95");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2760.53,6895.69");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4736.40,6814.79");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5681.77,6782.37");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=82.54,6709.15");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=499.89,6740.83");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1046.24,6757.33");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1784.77,6738.01");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2205.26,6517.73");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3213.28,6671.25");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3855.77,6590.98");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4396.57,6672.22");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5271.28,6521.00");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=829.88,6315.77");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1552.22,6319.70");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2552.99,6431.78");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3440.29,6387.81");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4886.04,6400.45");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5681.61,6312.49");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2138.51,6105.98");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3040.61,6205.46");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3856.25,6207.40");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4543.93,6108.06");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=388.47,5867.31");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1063.47,5953.28");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1652.25,5971.00");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2655.38,5904.32");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3751.51,5823.44");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4247.26,5854.37");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5460.26,5925.30");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=946.23,5612.06");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2300.48,5761.75");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2905.67,5591.78");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3267.67,5658.17");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4540.17,5576.25");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4915.95,5624.84");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5901.50,5762.70");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1278.62,5412.79");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1695.22,5452.88");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2101.95,5434.44");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3623.66,5412.08");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4083.67,5428.57");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5418.05,5524.41");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=875.02,5133.85");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1524.12,5110.67");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2472.28,5182.17");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3296.47,5256.92");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4153.26,5046.72");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5179.13,5254.66");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5832.78,5226.47");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=373.60,4951.75");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1926.69,4867.50");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4757.84,4924.27");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=893.11,4693.19");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1418.98,4702.39");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2369.19,4774.99");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3060.75,4659.67");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3500.22,4771.29");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4424.25,4712.38");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5212.12,4548.70");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5622.70,4704.45");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=562.27,4490.76");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2086.38,4466.95");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2465.01,4362.50");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3802.94,4510.71");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4742.03,4478.95");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=293.76,4049.27");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=666.26,4093.61");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1205.83,4221.99");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1847.29,4119.25");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2845.66,4154.45");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3424.64,4121.42");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3903.71,4064.32");
    }

}

