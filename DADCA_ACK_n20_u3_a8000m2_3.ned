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

network DADCA_ACK_n20_u3_a8000m2_3
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
        @display("i=misc/sensor;p=932.17,7837.44");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1477.54,7890.60");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2002.74,7993.19");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3010.95,7763.98");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3380.70,7924.11");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4339.87,7970.19");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5079.04,7854.20");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5685.25,7769.98");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=357.30,7618.86");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2056.88,7558.68");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2479.83,7523.59");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3719.39,7711.74");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4064.82,7556.11");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4562.50,7550.20");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5338.38,7546.82");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=78.15,7316.65");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1455.48,7277.46");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3374.44,7487.16");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4988.01,7478.54");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=371.12,7044.71");
    }

}

