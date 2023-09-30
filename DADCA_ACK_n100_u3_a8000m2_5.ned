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

network DADCA_ACK_n100_u3_a8000m2_5
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
        @display("i=misc/sensor;p=376.70,7856.50");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1403.97,7792.39");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3581.05,7810.65");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4231.67,7944.85");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4829.07,7996.30");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5992.36,7873.40");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=39.24,7588.01");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=959.41,7717.16");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1974.87,7699.69");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2867.64,7605.50");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4649.22,7641.95");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5298.66,7712.78");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=537.15,7479.39");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3188.35,7301.28");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3623.39,7305.54");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3988.14,7467.82");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4991.82,7341.01");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5814.82,7324.04");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=542.24,7048.34");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=936.22,7218.63");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1420.94,7097.47");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1834.60,7024.45");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4253.25,7213.23");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=151.84,6995.10");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=925.43,6845.43");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2468.37,6992.92");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3030.80,6771.39");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3849.45,6951.12");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4230.68,6859.41");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4845.45,6986.02");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5543.21,6954.57");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1250.32,6523.93");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1630.86,6731.41");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2236.27,6678.13");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2583.44,6628.15");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3547.01,6721.72");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4779.90,6528.65");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5801.72,6553.25");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=345.57,6294.72");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1874.01,6368.88");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4253.85,6438.71");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5287.85,6493.03");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=852.36,6139.82");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1590.75,6121.42");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2180.54,6064.72");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2586.53,6176.41");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2963.80,6215.97");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3657.26,6186.02");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4385.62,6102.36");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4973.62,6130.21");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5839.46,6095.53");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1129.80,5856.24");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2688.50,5791.99");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5392.43,5960.76");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=849.26,5625.79");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1566.17,5710.32");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2295.48,5653.87");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3105.50,5772.48");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3953.14,5563.66");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4441.06,5542.91");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5076.98,5703.04");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5644.30,5589.59");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1858.13,5435.97");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2629.83,5299.39");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3171.94,5374.48");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=105.82,5155.07");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1056.02,5203.00");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1465.52,5151.68");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1948.60,5055.51");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3555.87,5245.61");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3990.72,5190.56");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4908.91,5046.66");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5355.09,5193.10");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5980.23,5053.19");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=551.98,4871.16");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2583.59,4914.17");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3072.62,4915.37");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4580.84,4884.50");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5566.25,4858.52");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=18.30,4570.48");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=910.84,4722.34");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1861.93,4650.57");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3631.16,4725.71");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4226.10,4692.25");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5090.61,4583.86");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5933.07,4567.75");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=948.61,4334.14");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1272.82,4520.10");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2298.99,4517.48");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2656.04,4446.71");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3700.16,4341.75");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4761.21,4420.20");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=204.32,4120.72");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=630.76,4084.42");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1480.80,4047.77");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1792.19,4256.03");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3133.71,4260.17");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5446.72,4214.08");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5974.51,4191.11");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1138.57,3890.52");
    }

}

