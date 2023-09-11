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

network DADCA_ACK_n20_a8000m2_lpnFn1_1
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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=44.98,135.45");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=41.30,41.96");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=111.51,5.48");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=234.45,95.64");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=293.92,51.70");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=385.90,40.29");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=374.31,100.74");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=456.33,134.69");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=529.03,178.86");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=587.75,195.08");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=590.63,104.32");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=634.87,60.75");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=577.29,41.93");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=505.34,3.33");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=673.38,122.61");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=653.01,203.20");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=616.58,294.48");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=674.81,315.76");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=710.42,256.55");
    }

}

