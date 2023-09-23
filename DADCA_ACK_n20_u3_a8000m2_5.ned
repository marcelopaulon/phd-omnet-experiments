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

network DADCA_ACK_n20_u3_a8000m2_5
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
        @display("i=misc/sensor;p=188.94,7908.85");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=811.56,7857.85");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2468.03,7870.61");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2977.83,7846.67");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3390.41,7986.74");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3987.31,7965.11");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4360.75,7805.47");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5256.77,7973.66");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1969.69,7706.13");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4868.70,7686.76");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5754.20,7583.87");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=776.98,7390.88");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1137.00,7269.58");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2744.40,7416.14");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3110.63,7279.10");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3763.92,7410.09");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4526.58,7387.90");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5090.25,7405.57");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=267.69,7231.35");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1502.89,7226.26");
    }

}

