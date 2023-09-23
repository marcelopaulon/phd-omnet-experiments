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

network DADCA_ACK_n100_u4_a8000m2_8
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
        @display("i=misc/sensor;p=1338.17,7997.92");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2401.22,7902.05");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3151.12,7948.71");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3834.84,7860.52");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5500.58,7898.41");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=296.14,7665.95");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=766.32,7599.87");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1116.62,7606.54");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1477.32,7567.17");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1847.07,7744.60");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2798.09,7526.40");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4291.65,7699.20");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5999.30,7700.61");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2108.72,7502.98");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3376.09,7407.32");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4483.21,7362.52");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4868.44,7443.40");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5458.17,7496.49");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5761.30,7303.79");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=857.19,7165.65");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1298.22,7152.85");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1964.10,7153.47");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2388.70,7225.61");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3889.54,7186.57");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4872.44,7029.68");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=352.22,6882.00");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2547.88,6768.18");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2910.77,6952.08");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3413.31,6956.66");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4442.78,6900.60");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5233.20,7003.70");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=164.96,6553.79");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=624.28,6524.71");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1196.14,6691.58");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1696.13,6533.79");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2122.47,6543.35");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3857.62,6761.27");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4805.07,6601.43");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5546.85,6618.19");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2844.53,6435.31");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3502.85,6477.86");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4232.83,6471.76");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5165.66,6400.98");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5965.18,6462.44");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=579.48,6095.00");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1241.24,6204.25");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2245.97,6146.48");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3051.99,6058.93");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3489.07,6069.11");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5650.89,6075.28");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1363.24,5823.11");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1658.91,6018.86");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3808.91,5911.46");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4899.25,5804.67");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=152.59,5769.08");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=714.24,5618.02");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2314.17,5745.76");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2760.07,5752.12");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3296.28,5557.85");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4075.29,5628.49");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5315.44,5763.77");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1672.83,5514.91");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2068.30,5484.29");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3051.26,5278.77");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3915.19,5305.68");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4840.92,5322.14");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5945.42,5521.32");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=65.30,5253.21");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1207.16,5242.92");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2691.60,5226.15");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3563.37,5040.42");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4522.19,5148.24");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5358.08,5194.48");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=674.55,5005.50");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1490.46,4962.51");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3157.89,4862.02");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4103.81,4985.11");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4995.86,4833.68");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5842.06,4830.04");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=184.40,4582.59");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2235.29,4775.16");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2873.95,4612.87");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3799.49,4675.86");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=577.08,4494.74");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1671.03,4339.63");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3419.92,4529.40");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4009.19,4386.18");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4500.51,4501.62");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5066.47,4444.48");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1166.07,4266.58");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2382.53,4177.49");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3314.55,4188.39");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4306.57,4114.39");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4706.44,4126.18");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5741.11,4246.80");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=188.71,3993.97");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=710.59,3853.16");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1315.31,3933.96");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1695.54,3933.62");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2994.76,3940.99");
    }

}

