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

network DADCA_ACK_n20_u16_a8000m2_0
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
        @display("i=misc/sensor;p=195.94,7821.54");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=678.72,7888.53");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1213.80,7868.07");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1892.86,7798.36");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2248.12,7885.26");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2750.45,7850.04");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3201.91,7841.69");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3676.89,7947.01");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3533.17,7508.10");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3947.75,7686.80");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4318.71,7680.43");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4966.58,7700.39");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5643.06,7728.87");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=547.32,7451.01");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1146.06,7425.50");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1691.33,7494.26");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2061.21,7329.23");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2977.29,7360.24");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4554.79,7414.04");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5169.19,7405.45");
    }

}

