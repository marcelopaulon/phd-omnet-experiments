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

network DADCA_ACK_n100_u8_a8000m2_1
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
        @display("i=misc/sensor;p=916.85,7894.55");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2198.16,7964.94");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2968.97,7865.35");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3499.92,7769.30");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4129.34,7894.69");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5110.23,7972.62");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=135.54,7615.29");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1650.35,7704.90");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2610.19,7544.60");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5506.86,7680.80");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5890.30,7561.18");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=771.64,7425.46");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1175.09,7308.58");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2015.50,7309.47");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3028.03,7469.96");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3425.22,7309.87");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4399.41,7374.24");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5096.95,7381.37");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=174.32,7230.64");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=757.78,7014.71");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1766.72,7026.11");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2529.18,7168.15");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4734.40,7209.08");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5500.46,7127.34");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1104.16,6874.08");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2321.72,6829.25");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3006.43,6921.99");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3955.73,6854.65");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4418.27,6950.73");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5055.89,6796.88");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5726.40,6831.39");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=380.07,6607.10");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3354.97,6730.75");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4653.60,6525.65");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5331.36,6565.46");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1322.03,6319.62");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1727.47,6332.52");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2723.38,6495.61");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3992.82,6498.92");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4999.33,6323.22");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5668.22,6362.10");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=101.90,6031.37");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=790.09,6193.40");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2265.67,6090.53");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3008.15,6261.45");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3472.13,6163.88");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1045.24,5781.90");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1416.87,5820.23");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1906.95,5880.91");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2682.33,5999.27");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3923.06,5998.09");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4284.34,5880.45");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4811.11,5836.39");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5245.33,5920.94");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=142.88,5553.52");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=457.81,5706.79");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2473.90,5604.09");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2932.82,5543.70");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3423.54,5716.23");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3841.06,5647.92");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5130.89,5574.40");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5654.44,5717.71");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=667.21,5301.57");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1648.22,5388.47");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3580.22,5328.56");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=73.70,5201.38");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1159.14,5177.69");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1834.75,5063.28");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2182.68,5141.26");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2668.00,5151.97");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4228.73,5225.36");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4668.02,5124.09");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5147.94,5073.41");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5710.53,5075.20");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=480.59,4783.87");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3227.16,4891.64");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=122.86,4710.60");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=959.99,4651.24");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1624.27,4729.58");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2375.22,4639.92");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3872.67,4695.14");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4485.14,4698.80");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5033.80,4728.56");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1279.83,4475.02");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1977.76,4450.78");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2653.52,4396.21");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3126.50,4298.04");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3893.44,4294.21");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4258.43,4305.86");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5352.17,4514.38");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=731.51,4200.59");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1970.47,4094.63");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2324.82,4207.65");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3529.21,4171.71");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4869.53,4135.12");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=25.24,3836.76");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1305.99,3979.51");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1709.17,3810.59");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2768.12,3832.16");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4093.03,3938.32");
    }

}

