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

network DADCA_ACK_n20_u2_a8000m2_5
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
        @display("i=misc/sensor;p=987.19,7941.40");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1414.48,7901.47");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2398.53,7783.75");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2755.75,7875.72");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3270.47,7761.33");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3982.44,7926.35");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4422.27,7985.84");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4761.53,7769.83");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5589.63,7869.59");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=207.56,7634.52");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1040.54,7521.18");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1598.89,7518.48");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5969.12,7700.40");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=571.83,7437.24");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2215.98,7272.20");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2898.87,7396.16");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4318.69,7416.52");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4984.49,7471.16");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5645.72,7459.48");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=538.26,7020.78");
    }

}

