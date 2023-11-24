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

network DADCA_ACK_n50_u8_a8000m2_8
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
        @display("i=misc/sensor;p=233.81,7841.36");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1286.35,7937.20");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1849.19,7889.23");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2693.45,7896.38");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3656.92,7842.43");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4985.72,7777.05");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5416.91,7857.55");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=531.34,7547.42");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=935.01,7697.51");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2096.31,7629.31");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3010.10,7650.05");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4361.83,7690.65");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5628.02,7574.74");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5973.49,7505.87");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=199.59,7381.00");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1599.26,7271.80");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2373.12,7304.28");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3318.53,7433.98");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4004.38,7403.03");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4990.23,7393.88");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5422.02,7258.82");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=960.68,7195.62");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2904.58,7254.22");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3295.60,7062.38");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3664.07,7055.07");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4562.45,7109.60");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5979.81,7024.39");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=164.18,6790.79");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=595.54,7001.41");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1318.07,6965.44");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1962.78,6866.51");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2563.58,6781.67");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5184.43,6827.87");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=872.58,6625.16");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1663.16,6561.96");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3742.92,6654.74");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4297.92,6756.71");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4863.11,6657.22");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5726.59,6590.95");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1125.00,6297.39");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2186.28,6419.77");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3154.96,6500.50");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=66.71,6126.33");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=476.81,6259.87");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1653.20,6121.85");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2631.36,6179.20");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3745.11,6232.78");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4581.84,6128.23");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5208.76,6147.65");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2016.89,5913.91");
    }

}

