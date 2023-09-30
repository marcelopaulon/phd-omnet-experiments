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

network DADCA_ACK_n50_u8_a8000m2_9
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
        @display("i=misc/sensor;p=1213.35,7808.26");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1813.46,7942.08");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3098.35,7949.11");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4552.04,7910.27");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4952.84,7910.52");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5778.05,7817.36");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=126.72,7744.88");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=812.92,7646.78");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2183.11,7634.09");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3641.84,7643.53");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4303.66,7652.54");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5356.63,7557.22");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1185.85,7374.26");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1757.64,7434.15");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2958.96,7350.48");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4112.58,7304.95");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4959.90,7489.93");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5721.64,7445.67");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=183.93,7052.04");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=531.15,7234.10");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2324.57,7251.90");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3735.33,7240.83");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4770.49,7088.04");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5483.73,7086.25");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=769.99,6783.98");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1346.02,6834.12");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1837.37,6844.30");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2616.16,6863.99");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3167.52,6990.08");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3907.57,6821.49");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4323.37,6955.88");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5897.66,6903.02");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=390.36,6761.33");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1585.48,6541.90");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2949.10,6691.62");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3544.02,6688.61");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4703.77,6712.72");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5114.55,6750.19");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=946.36,6351.45");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2268.47,6440.40");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3225.51,6295.56");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4206.16,6320.53");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4554.00,6393.71");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5059.48,6287.41");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5428.59,6287.28");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5765.86,6467.51");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=371.73,6263.63");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2012.37,6053.02");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2625.47,6214.66");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3863.90,6096.68");
    }

}

