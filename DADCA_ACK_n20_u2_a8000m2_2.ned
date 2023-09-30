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

network DADCA_ACK_n20_u2_a8000m2_2
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
        @display("i=misc/sensor;p=431.43,7813.98");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1391.83,7764.99");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2015.16,7993.77");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2652.94,7987.67");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4785.94,7906.01");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5255.14,7875.48");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=906.54,7681.80");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2327.69,7664.48");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3018.84,7678.26");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3513.52,7631.73");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4195.54,7627.21");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4878.92,7551.41");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2123.75,7326.95");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2650.12,7378.23");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5428.78,7444.03");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=960.20,7245.16");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1680.18,7021.19");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3330.97,7031.39");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3760.95,7010.74");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4209.83,7234.19");
    }

}

