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

network DADCA_ACK_n20_u16_a8000m2_9
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
        @display("i=misc/sensor;p=14.07,7884.27");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=503.07,7787.59");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=907.90,7907.26");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2116.76,7941.76");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2568.63,7816.94");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3628.61,7993.56");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4874.91,7988.91");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5531.89,7881.27");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1145.96,7570.49");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1704.12,7549.21");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2269.32,7606.48");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3659.59,7505.74");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4234.16,7559.97");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5895.14,7662.93");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=562.54,7282.68");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1958.91,7301.14");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2612.99,7449.24");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2993.96,7459.88");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4824.50,7419.22");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5246.16,7324.22");
    }

}

