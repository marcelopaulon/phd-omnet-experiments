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

network DADCA_ACK_n100_u2_a8000m2_7
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
        @display("i=misc/sensor;p=1570.21,7790.46");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2732.19,7948.09");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3176.36,7825.08");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5853.45,7979.30");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=82.54,7696.30");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=470.58,7737.98");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=938.90,7531.79");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4029.39,7549.43");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4502.70,7595.57");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4946.88,7709.26");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5997.59,7538.04");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=555.36,7276.86");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1465.42,7293.03");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1988.93,7406.93");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2457.81,7455.81");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2992.47,7439.25");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3685.04,7411.76");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5618.17,7433.96");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2174.72,7095.24");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2542.06,7098.01");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3225.42,7112.05");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3666.13,7060.64");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4241.78,7051.60");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4685.11,7073.79");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5219.83,7253.45");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5838.14,7039.25");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=191.07,6952.12");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=982.69,6932.33");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1617.13,6824.36");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5044.98,6949.97");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=36.45,6622.34");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=423.22,6571.39");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2918.00,6622.37");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3543.01,6643.59");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4207.02,6665.03");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4557.32,6636.58");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5165.28,6587.64");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5897.79,6655.28");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=763.25,6300.86");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1128.36,6337.28");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1510.70,6399.06");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2205.38,6449.01");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4856.23,6367.98");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5498.03,6397.55");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1793.56,6149.87");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2520.05,6092.23");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3278.64,6240.73");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4257.39,6253.22");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4577.18,6106.80");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5215.08,6123.36");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5682.13,6092.83");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=408.24,5913.21");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=755.25,5839.08");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1374.05,5968.70");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2804.52,5854.81");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3826.77,5999.88");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=273.99,5544.90");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1086.54,5529.13");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1731.51,5651.06");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2229.37,5681.59");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3216.70,5729.22");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3584.64,5640.56");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4290.45,5611.72");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4706.56,5662.63");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5216.48,5726.98");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=718.28,5307.47");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1449.24,5397.81");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2545.95,5501.82");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3124.26,5342.45");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3522.56,5279.20");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4934.05,5384.85");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5585.26,5444.78");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1222.61,5040.64");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1797.33,5081.93");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2241.49,5128.08");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3980.08,5216.51");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4668.60,5042.94");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5021.72,5037.54");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5998.95,5092.67");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=685.40,4895.29");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2771.43,4993.86");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4387.11,4815.22");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5370.71,4889.46");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=332.10,4763.52");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1068.68,4713.20");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1527.31,4737.30");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1892.85,4647.18");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3187.96,4721.74");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3694.58,4616.28");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4754.79,4687.41");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5946.99,4588.27");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1559.42,4312.45");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2388.41,4515.11");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3217.71,4368.95");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4183.21,4477.60");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4521.81,4349.03");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5213.34,4509.55");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5566.53,4349.14");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=838.00,4282.31");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1267.37,4109.29");
    }

}

