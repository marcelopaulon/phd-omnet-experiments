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
        @display("i=misc/sensor;p=188.82,7800.39");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2758.69,7763.15");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3208.75,7944.15");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3693.13,7934.04");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4430.90,7962.67");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5103.36,7908.67");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=745.15,7697.73");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2381.82,7676.05");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3042.44,7552.26");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5446.62,7650.67");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=412.59,7357.05");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1431.54,7469.35");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1988.17,7497.53");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2513.46,7350.85");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3501.95,7398.05");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4171.68,7504.29");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4783.27,7446.17");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=237.30,7034.08");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=869.65,7178.36");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2197.75,7037.22");
    }

}

