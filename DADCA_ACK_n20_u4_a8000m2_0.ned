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

network DADCA_ACK_n20_u4_a8000m2_0
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
        @display("i=misc/sensor;p=26.16,7816.81");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=623.60,7952.68");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1273.98,7866.84");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2653.08,7953.90");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3155.49,7963.26");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3634.95,7874.03");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4237.77,7785.59");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4764.85,7922.32");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5355.17,7923.16");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=471.00,7505.04");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2113.00,7682.30");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5032.94,7598.29");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5913.34,7715.66");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=926.30,7475.43");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1614.44,7348.19");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2142.38,7331.45");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2887.41,7433.76");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3368.94,7266.22");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3844.59,7399.74");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4634.02,7295.17");
    }

}

