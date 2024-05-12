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

network DADCA_ACK_n20_u3_a8000m2_0
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
        @display("i=misc/sensor;p=67.97,7872.88");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=429.65,7880.33");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1406.20,7967.43");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3568.71,7891.35");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3970.39,7890.80");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5274.20,7953.67");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1656.15,7647.06");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2240.27,7692.69");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2740.09,7712.87");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3301.19,7579.03");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4324.94,7741.87");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4975.89,7636.07");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=674.63,7299.13");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1255.85,7327.45");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1954.26,7268.78");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2532.04,7420.41");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3663.25,7476.70");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4054.81,7487.36");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5392.51,7384.15");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=299.07,7114.50");
    }

}

