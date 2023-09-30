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

network DADCA_ACK_n20_u4_a8000m2_3
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
        @display("i=misc/sensor;p=213.49,7759.21");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=701.49,7915.14");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3345.12,7877.10");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3805.15,7901.99");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4556.14,7785.32");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5939.39,7973.00");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1806.49,7539.49");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2267.86,7535.98");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2716.02,7525.15");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3433.81,7531.70");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4206.47,7675.67");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4884.21,7508.95");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5219.64,7622.61");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5699.47,7681.24");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=480.40,7262.71");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1118.12,7495.69");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3094.83,7437.95");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3972.94,7401.74");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4479.62,7259.15");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5548.14,7272.18");
    }

}

