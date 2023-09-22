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

network DADCA_ACK_n100_a8000m2_3
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
        @display("i=misc/sensor;p=1543.82,7785.91");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2192.26,7987.93");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2558.30,7972.56");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3445.89,7982.76");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5269.52,7956.38");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=148.60,7666.39");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=515.14,7628.94");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3981.72,7584.91");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4992.71,7739.18");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5605.06,7582.31");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5922.72,7729.99");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1340.32,7398.34");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1912.90,7504.54");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2455.06,7320.47");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2883.58,7457.74");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3328.85,7384.92");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4749.17,7425.67");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5176.37,7289.34");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5952.94,7349.97");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=239.77,7213.30");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=818.28,7181.20");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1197.39,7067.54");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1906.00,7055.68");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2952.40,7067.48");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4100.56,7238.80");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4499.21,7017.41");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=547.57,6832.37");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3298.31,6918.41");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3869.22,6961.51");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5000.05,6944.50");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5618.70,6975.11");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1021.57,6637.47");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1629.03,6706.35");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2768.01,6701.01");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3370.07,6520.79");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5376.91,6579.82");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5832.83,6612.47");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=325.01,6351.52");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1231.18,6322.43");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2285.66,6488.80");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3918.62,6353.79");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4352.29,6492.45");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4831.72,6370.98");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=693.80,6110.75");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1726.64,6145.22");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2221.89,6052.73");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2905.30,6202.34");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3574.77,6093.22");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5268.59,6227.71");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5977.45,6133.53");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3785.50,5808.87");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4367.50,5869.51");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4736.55,5848.38");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5095.94,5850.45");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5629.08,5910.05");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=318.85,5587.79");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1328.05,5673.22");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1811.81,5558.53");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2613.99,5752.09");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3291.68,5658.48");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=686.19,5427.51");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2151.53,5332.13");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2785.11,5391.39");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3265.67,5305.36");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3945.71,5352.30");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4390.78,5392.07");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4933.20,5461.62");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5504.23,5442.01");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=352.76,5219.09");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1067.34,5243.78");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1492.35,5241.37");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2607.05,5084.07");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4682.63,5190.75");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=975.28,4884.15");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3792.91,4828.47");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5154.11,4805.07");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5704.43,4841.75");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=677.29,4678.20");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1606.30,4720.16");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2742.39,4704.01");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3159.76,4743.33");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4151.30,4730.29");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4858.66,4615.37");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=631.54,4287.95");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2078.69,4499.68");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2733.61,4306.72");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3282.19,4332.03");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5153.94,4419.09");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5516.96,4440.69");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5899.83,4440.31");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1004.77,4192.69");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1494.78,4058.48");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3783.36,4090.73");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4182.18,4072.47");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4635.92,4082.75");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4996.10,4096.54");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5611.31,4062.83");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=408.24,3926.10");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1924.60,4013.74");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2347.90,3875.54");
    }

}

