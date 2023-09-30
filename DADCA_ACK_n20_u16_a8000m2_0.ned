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
        @display("i=misc/sensor;p=709.12,7918.89");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3991.89,7804.72");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4406.78,7798.08");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5571.68,7994.54");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1024.63,7534.10");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1676.86,7692.95");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2409.32,7678.52");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2904.77,7663.22");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3681.75,7589.69");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5134.61,7726.09");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5822.06,7699.58");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=187.02,7394.65");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1613.29,7338.04");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1977.17,7369.24");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2816.63,7320.56");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3321.53,7437.30");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4249.71,7400.49");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4902.56,7317.88");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=497.43,7126.71");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=881.50,7197.49");
    }

}

