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

network DADCA_ACK_n20_u8_a8000m2_0
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
        @display("i=misc/sensor;p=307.64,7828.39");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=980.92,7893.36");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1357.39,7936.76");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2550.63,7949.89");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3138.44,7969.56");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2159.09,7674.93");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2808.58,7623.56");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3646.98,7589.97");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4396.10,7531.87");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4795.44,7688.67");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5564.36,7748.83");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5844.96,7536.25");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=391.26,7340.83");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=978.21,7380.55");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1423.30,7456.74");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2585.12,7305.86");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3015.38,7323.42");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4068.62,7394.58");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5063.66,7386.06");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=866.49,7033.03");
    }

}

