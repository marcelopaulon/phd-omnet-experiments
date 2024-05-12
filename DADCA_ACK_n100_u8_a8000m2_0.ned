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
        @display("i=misc/sensor;p=752.39,7979.70");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1173.24,7896.41");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1778.20,7897.12");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4169.61,7971.42");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2650.34,7696.63");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3372.76,7716.03");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4724.99,7579.66");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=758.35,7495.25");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1273.88,7502.01");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1688.09,7269.29");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2060.19,7451.53");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2974.78,7435.04");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3884.19,7342.49");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4352.35,7306.29");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1314.21,7120.39");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2296.22,7117.20");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2662.96,7238.38");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5037.32,7038.51");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5472.34,7077.38");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=94.40,6909.56");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=609.12,6962.49");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=999.60,6822.22");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2952.28,6993.99");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3361.32,6974.71");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4029.23,6956.41");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4673.80,6827.80");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=649.74,6521.05");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1417.66,6704.21");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2424.07,6740.18");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3471.12,6600.96");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3916.01,6618.70");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4272.20,6632.29");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5245.68,6691.46");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5570.70,6558.20");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=62.08,6499.80");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2066.29,6483.50");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2767.77,6470.76");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3229.00,6346.81");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4128.76,6303.95");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4572.82,6369.70");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=413.34,6189.64");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=805.18,6037.92");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1316.69,6227.21");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3698.04,6049.16");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5358.37,6029.92");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1948.74,5982.05");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2403.05,6005.96");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2923.50,5914.48");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3346.96,6012.53");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4296.59,5933.02");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=321.64,5690.48");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=652.74,5564.49");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1004.21,5623.62");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2055.05,5526.72");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3781.89,5587.29");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4841.56,5540.19");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=287.89,5328.28");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1587.59,5296.81");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3127.19,5352.45");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4356.74,5505.39");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5220.21,5290.42");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=640.09,5062.49");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1155.60,5181.48");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2578.30,5129.48");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3951.39,5068.01");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4831.78,5124.43");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5924.42,5039.25");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2244.87,4949.61");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3054.67,4860.17");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3540.93,4920.13");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5270.58,4939.23");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=153.04,4742.85");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=590.84,4618.19");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=952.67,4574.70");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1453.78,4675.95");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1995.99,4616.66");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2404.12,4632.65");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2789.77,4575.13");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3940.31,4638.96");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4334.57,4763.99");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=398.09,4304.84");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1338.82,4343.38");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3306.19,4512.33");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4225.35,4334.04");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4590.51,4448.93");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4987.76,4507.48");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5728.70,4424.98");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1853.70,4264.87");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3309.77,4075.04");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3622.05,4246.58");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4562.07,4053.13");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5962.56,4137.21");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=139.40,3962.33");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=603.63,3898.85");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1096.24,3920.73");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1965.95,3836.43");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2458.14,3985.02");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2955.50,3971.38");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4845.74,3816.52");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5207.26,3989.59");
    }

}

