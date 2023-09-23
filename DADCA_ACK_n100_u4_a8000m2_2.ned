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

network DADCA_ACK_n100_u4_a8000m2_2
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
        @display("i=misc/sensor;p=1015.12,7791.41");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1823.15,7858.59");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3137.89,7776.77");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5044.96,7897.59");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=424.64,7618.82");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1506.97,7646.83");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2118.03,7594.56");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2622.52,7572.37");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3693.18,7608.20");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4203.93,7654.09");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5613.34,7723.01");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3025.85,7400.95");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4188.50,7274.67");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4654.22,7411.97");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5701.59,7303.21");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=413.29,7148.61");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1014.98,7186.71");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1586.48,7245.00");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1932.30,7089.72");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5033.55,7122.96");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=82.35,6866.34");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=730.82,6837.08");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1112.59,6815.72");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2369.01,6990.87");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3082.13,6903.18");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3485.86,6914.96");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3900.71,6979.40");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4563.37,6807.98");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5812.82,6896.99");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=320.67,6541.90");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1446.95,6578.69");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2033.88,6536.50");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2685.43,6664.57");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4205.61,6761.16");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5083.78,6519.59");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5490.20,6663.44");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=955.71,6303.88");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2428.11,6285.41");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2866.02,6273.70");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3253.40,6479.24");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3952.23,6493.01");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=560.56,6080.46");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1962.88,6067.56");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3865.94,6043.37");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4550.92,6170.21");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5405.16,6040.04");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5782.59,6125.30");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1264.11,5923.93");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3529.29,5845.38");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4273.37,5890.83");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=892.85,5621.09");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1905.55,5593.86");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2477.03,5728.06");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4824.31,5632.95");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5513.13,5544.18");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5843.41,5694.51");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=348.41,5455.66");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3217.33,5451.20");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3871.44,5388.23");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4545.23,5390.63");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=459.83,5099.58");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=850.88,5118.64");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1412.76,5251.25");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1827.83,5242.79");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2256.15,5233.62");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2935.58,5202.96");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3570.39,5115.22");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5000.73,5150.85");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5403.50,5072.50");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5745.54,5150.57");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=6.91,4900.94");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1191.73,4803.69");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1967.07,4919.17");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2605.51,5010.11");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4249.25,4979.54");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4686.76,4962.43");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5686.74,4796.70");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=611.97,4751.15");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1620.14,4623.27");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2855.96,4600.46");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3976.56,4554.95");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4469.67,4564.43");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=280.77,4530.66");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=816.78,4447.53");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1206.76,4369.37");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2112.20,4520.05");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2615.92,4304.92");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3298.20,4290.76");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4854.77,4467.05");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5299.89,4305.48");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=24.66,4236.60");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2946.60,4188.09");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3798.43,4240.19");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4296.30,4057.25");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4953.52,4089.70");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5886.26,4193.99");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=316.45,4029.36");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1554.67,3984.10");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2032.78,3824.80");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2689.37,3903.09");
    }

}

