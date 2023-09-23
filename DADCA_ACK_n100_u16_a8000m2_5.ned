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
        @display("i=misc/sensor;p=836.92,7913.25");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2759.95,7925.82");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3193.68,7803.30");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3803.66,7984.93");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4422.61,7812.03");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5545.71,7972.73");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5829.06,7765.94");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=373.07,7726.80");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1496.37,7730.10");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1949.06,7668.50");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2330.38,7709.96");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2935.12,7515.99");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3775.01,7623.80");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4892.72,7726.67");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5248.28,7712.02");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=39.98,7501.31");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=614.55,7423.28");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2100.09,7346.19");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3442.49,7456.48");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4846.76,7271.74");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5450.10,7291.70");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5993.86,7270.13");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=189.93,7121.86");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1150.23,7210.98");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2546.53,7222.78");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3927.98,7127.80");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4294.69,7066.74");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=662.83,6989.36");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1718.73,6913.13");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2848.11,7009.50");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3287.80,7001.19");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4573.10,6800.72");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4961.58,6776.62");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5816.84,6917.55");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=519.76,6646.87");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=952.63,6551.60");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1334.79,6695.33");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2098.95,6722.85");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2606.16,6679.28");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3135.10,6624.94");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4170.69,6698.41");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5291.29,6537.66");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1424.29,6339.46");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2293.32,6381.56");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2738.96,6294.00");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3574.14,6478.54");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4726.18,6443.85");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2.79,6031.61");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=367.51,6141.46");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1967.13,6162.70");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3165.83,6098.79");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3532.15,6095.14");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4113.37,6105.75");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4599.37,6114.29");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5154.71,6202.82");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5563.19,6077.21");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=963.20,5860.92");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1394.55,5783.16");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2525.21,5947.46");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5056.04,5778.83");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=49.52,5537.75");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=502.50,5604.84");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1869.99,5724.32");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2439.41,5585.25");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2840.37,5541.33");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3136.61,5728.76");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4771.71,5525.28");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5698.56,5610.72");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=904.12,5380.62");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3780.01,5520.35");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4130.06,5440.10");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1320.93,5256.56");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1692.21,5081.40");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3464.36,5268.73");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4295.13,5097.91");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5181.78,5251.78");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5791.59,5249.30");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=442.74,4928.64");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=924.45,4857.85");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2498.86,5018.93");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3174.92,4997.07");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3687.19,4830.72");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4725.93,4832.62");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5422.77,4881.39");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1332.64,4695.11");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2658.57,4574.68");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4372.92,4740.51");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=462.23,4354.29");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=800.62,4506.08");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1945.87,4446.71");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3210.64,4458.11");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3778.76,4450.79");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4881.95,4434.69");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5768.28,4430.67");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1020.40,4171.23");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1542.78,4129.19");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2512.90,4077.03");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3552.57,4100.09");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4183.86,4064.88");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4576.33,4120.52");
    }

}

