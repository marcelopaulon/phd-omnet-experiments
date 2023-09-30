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

network DADCA_ACK_n20_u16_a8000m2_3
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
        @display("i=misc/sensor;p=1421.92,7939.16");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2037.55,7871.40");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2585.90,7953.66");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3388.60,7930.25");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3990.60,7780.42");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5359.77,7902.22");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=196.21,7674.11");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=644.67,7523.52");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2997.08,7641.50");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4779.20,7674.03");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1101.31,7282.86");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2404.55,7316.10");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3383.93,7371.36");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3911.92,7428.54");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4478.51,7362.08");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5504.86,7473.92");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5893.95,7360.23");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=366.24,7205.95");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1487.49,7161.95");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1870.05,7122.15");
    }

}

