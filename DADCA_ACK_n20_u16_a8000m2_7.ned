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

network DADCA_ACK_n20_u16_a8000m2_7
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
        @display("i=misc/sensor;p=265.07,7963.68");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=819.33,7967.79");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1779.74,7876.70");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2676.09,7782.64");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3398.08,7937.20");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=48.84,7529.71");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=494.44,7676.73");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1416.32,7536.25");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2089.81,7697.26");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2416.74,7541.78");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3400.38,7565.29");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4443.85,7606.62");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4848.25,7709.62");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5810.45,7656.97");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=932.90,7490.22");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1795.41,7478.98");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2899.88,7502.50");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4019.11,7355.75");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5310.64,7440.22");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5791.78,7306.85");
    }

}

