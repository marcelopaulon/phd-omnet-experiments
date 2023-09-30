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

network DADCA_ACK_n50_u8_a8000m2_6
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
        @display("i=misc/sensor;p=1330.17,7960.02");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1696.78,7991.60");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2541.42,7945.35");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3446.87,7950.81");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3993.56,7956.28");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4517.41,7901.31");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2047.65,7623.37");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2419.27,7578.39");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2834.74,7620.67");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3205.52,7636.67");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5523.04,7710.41");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=392.65,7311.50");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=954.63,7502.67");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1578.65,7405.03");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3798.44,7478.17");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4212.91,7434.93");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1033.13,7117.19");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1861.98,7036.37");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2230.40,7072.17");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2841.23,7173.82");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3210.43,7110.61");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3543.68,7226.79");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4016.42,7052.41");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4758.64,7131.72");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5139.45,7248.09");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5685.51,7017.84");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=29.36,6796.97");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=781.71,6868.39");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1453.56,6861.08");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4352.25,6945.29");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=379.84,6604.09");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2076.47,6648.81");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2484.27,6651.30");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3077.42,6753.28");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3365.92,6519.06");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3738.54,6637.23");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4714.53,6743.90");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5264.91,6661.99");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5994.71,6635.18");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1231.65,6506.78");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1743.91,6473.58");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2281.17,6325.81");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4260.13,6356.37");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5679.22,6275.17");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=631.51,6223.83");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=998.14,6240.74");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1779.49,6066.15");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2938.06,6252.94");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3302.51,6099.41");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3900.46,6073.91");
    }

}

