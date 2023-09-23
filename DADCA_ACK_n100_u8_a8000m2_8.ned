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

network DADCA_ACK_n100_u8_a8000m2_8
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
        @display("i=misc/sensor;p=271.73,7902.92");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=670.81,7971.05");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2667.75,7983.58");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3758.48,7759.71");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4523.01,7763.65");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4986.97,7886.38");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5588.56,7835.23");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1275.24,7621.19");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1635.79,7651.42");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2246.60,7712.95");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2950.84,7695.56");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4278.77,7506.27");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5168.84,7522.09");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=31.22,7291.55");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=588.47,7339.53");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2187.95,7327.87");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2645.98,7376.55");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3288.53,7466.29");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4668.85,7345.26");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=915.59,7111.99");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1294.37,7155.98");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1839.08,7237.33");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2972.05,7200.79");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3575.10,7018.51");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3915.81,7178.57");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5372.15,7211.57");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=411.38,6862.21");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1083.74,6763.46");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1663.98,6889.27");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2091.72,6841.61");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2503.58,7000.90");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2866.92,6837.96");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4189.54,6901.59");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4799.35,6840.10");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5659.83,6966.40");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2469.44,6633.44");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3114.71,6539.28");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3746.71,6522.94");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5377.02,6683.90");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5908.54,6606.49");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1.09,6366.77");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=677.83,6282.30");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2022.75,6293.93");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4090.45,6320.25");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4623.52,6452.81");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1561.35,6214.91");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2322.90,6074.70");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2809.20,6198.62");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3485.32,6170.72");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5709.50,6027.70");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=98.46,5950.22");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=493.00,5964.22");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=940.50,5805.55");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1420.69,5834.52");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3141.00,5978.17");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3900.92,5861.17");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5293.78,5899.84");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2046.12,5723.28");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2383.55,5590.88");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4306.20,5749.38");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4670.97,5765.62");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=80.27,5506.72");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1006.59,5375.31");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1492.77,5420.75");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1884.67,5405.61");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3067.20,5332.09");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3470.33,5393.08");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4125.54,5282.81");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4533.25,5291.69");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4887.58,5452.88");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5588.81,5470.25");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=369.33,5198.51");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1297.15,5050.16");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1646.86,5101.48");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2716.62,5232.69");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5283.81,5147.22");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5848.63,5222.28");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=979.10,4792.46");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2109.45,4954.69");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2768.83,4863.17");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3137.08,4813.27");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3841.69,4840.38");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5849.08,4843.80");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=515.02,4645.56");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1601.35,4711.84");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2045.41,4604.42");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3563.98,4609.52");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4293.86,4759.85");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=944.02,4439.23");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2600.97,4510.24");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5245.41,4508.40");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5839.38,4421.60");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=426.26,4090.14");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1435.36,4282.02");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2329.98,4167.74");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2714.77,4062.51");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3148.61,4285.74");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4231.62,4283.26");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4733.67,4116.88");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5306.99,4044.99");
    }

}

